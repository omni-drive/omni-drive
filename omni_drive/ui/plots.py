import math
import time
from collections.abc import Callable, Sequence
from typing import Literal

import numpy as np
import pandas as pd
from bokeh.layouts import column
from bokeh.models import ColumnDataSource, DataRange1d, Div, HoverTool, LayoutDOM, Range1d
from bokeh.plotting import figure, show

from omni_drive.localization import Localizer
from omni_drive.robot import OmniDrive
from omni_drive.sensors import UltrasonicSensor
from omni_drive.ui.live import LiveView, Throttle
from omni_drive.ui.style import (
    ESTIMATES,
    KALMAN,
    MEASURED_COLOR,
    MUTED_COLOR,
    PLOT_TOOLS,
    SeriesPalette,
    dot,
    finish_figure,
    time_series_figure,
)
from omni_drive.ui.top_view import to_plot, top_view_figure

FACING_BY_DEGREES = {0: "front", 90: "left", -90: "right", 180: "rear", -180: "rear"}
NO_ECHO_TOLERANCE_M = 1e-6


def rmse(a, b) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    finite = np.isfinite(a) & np.isfinite(b)
    return float(np.sqrt(np.mean((a[finite] - b[finite]) ** 2))) if finite.any() else math.nan


def median_absolute_gap(a, b) -> float:
    gaps = np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))
    gaps = gaps[np.isfinite(gaps)]
    return float(np.median(gaps)) if gaps.size else math.nan


def read_number(channel: Callable[[], object]) -> float:
    try:
        value = channel()
    except Exception:
        return math.nan
    if value is None:
        return math.nan
    return float(getattr(value, "meters", value))


def sliding_time_range(window: float) -> DataRange1d:
    return DataRange1d(follow="end", follow_interval=window, range_padding=0)


def rollover_for(window: float, fps: float) -> int:
    return math.ceil(window * fps) + 1


class LivePlot(LiveView):
    def __init__(
        self,
        title: str = "",
        y_label: str = "",
        window: float = 20.0,
        fps: float = 10,
        y_range: tuple[float, float] | None = None,
        width: int = 820,
        height: int = 260,
    ):
        super().__init__(fps)
        self.window = window
        self._channels: dict[str, Callable[[], object]] = {}
        self._rows: list[dict[str, float]] = []
        self._t0: float | None = None
        self._palette = SeriesPalette()
        self.source = ColumnDataSource(data={"t": []})
        self.figure = time_series_figure(
            title, y_label, width, height, x_range=sliding_time_range(window)
        )
        if y_range is not None:
            self.figure.y_range = Range1d(*y_range)

    def add(
        self,
        name: str,
        fn: Callable[[], object],
        color: str | None = None,
        kind: Literal["line", "dots"] = "line",
    ) -> "LivePlot":
        if name in self._channels or name == "t":
            raise ValueError(f"Channel {name!r} already exists")
        color = color or self._palette.color_for(name, measured=kind == "dots")
        self._channels[name] = fn
        self.source.data = {**self.source.data, name: [math.nan] * len(self.source.data["t"])}
        if kind == "dots":
            renderer = self.figure.scatter(
                "t", name, source=self.source, size=6, color=color, legend_label=name
            )
        elif kind == "line":
            renderer = self.figure.line(
                "t", name, source=self.source, line_width=2, color=color, legend_label=name
            )
        else:
            raise ValueError(f"kind must be 'line' or 'dots', not {kind!r}")
        self.figure.add_tools(
            HoverTool(
                renderers=[renderer], tooltips=[(name, f"@{{{name}}}{{0.000}}"), ("t", "@t{0.0} s")]
            )
        )
        finish_figure(self.figure)
        return self

    @property
    def data(self) -> pd.DataFrame:
        return pd.DataFrame(self._rows, columns=["t", *self._channels])

    def clear(self) -> None:
        self._rows.clear()
        self._t0 = None
        self.source.data = {column: [] for column in self.source.data}

    def sample(self) -> dict[str, float]:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        row = {"t": now - self._t0} | {name: read_number(fn) for name, fn in self._channels.items()}
        self._rows.append(row)
        return row

    def _layout(self) -> LayoutDOM:
        return self.figure

    def _draw(self) -> None:
        row = self.sample()
        self.source.stream(
            {column: [value] for column, value in row.items()},
            rollover=rollover_for(self.window, self.fps),
        )


class UltrasonicView(LiveView):
    def __init__(
        self,
        robot: OmniDrive | None = None,
        localizer: Localizer | None = None,
        sensors: Sequence[UltrasonicSensor] | None = None,
        window: float = 30.0,
        fps: float = 10,
        width: int = 820,
        height: int = 200,
        y_max: float = 1.2,
        stats_refresh_s: float = 0.5,
    ):
        super().__init__(fps)
        self.localizer = localizer or (
            Localizer.for_robot(robot) if robot is not None else Localizer.default()
        )
        self.robot = self.localizer.robot
        self.sensors = list(sensors or self.robot.ultrasonic_sensors)
        self.window = window
        self._rows: list[dict[str, object]] = []
        self._last_sample_t: float | None = None
        self._stats_throttle = Throttle(stats_refresh_s)
        columns = ("t", "measured", *(style.key for style in ESTIMATES))
        self.sources = [ColumnDataSource(data={c: [] for c in columns}) for _ in self.sensors]
        time_range = sliding_time_range(window)
        self.figures = [
            self._sensor_figure(sensor, source, time_range, width, height, y_max)
            for sensor, source in zip(self.sensors, self.sources, strict=True)
        ]
        self.stats = Div(text="Waiting for LiDAR data…", width=width)

    @property
    def data(self) -> pd.DataFrame:
        columns = ["t", "sensor", "measured", *(style.key for style in ESTIMATES)]
        return pd.DataFrame(self._rows, columns=columns)

    def sample(self) -> list[dict[str, object]] | None:
        estimate = self.localizer.estimate
        if estimate is None or estimate.lidar is None or estimate.t == self._last_sample_t:
            return None
        self._last_sample_t = estimate.t
        poses = {style.key: getattr(estimate, style.key) for style in ESTIMATES}
        rows = [
            {"t": estimate.t, "sensor": sensor.sensor_id.name, "measured": read_number(sensor.read)}
            | {key: self.localizer.expected_range(sensor, pose) for key, pose in poses.items()}
            for sensor in self.sensors
        ]
        self._rows.extend(rows)
        return rows

    def _layout(self) -> LayoutDOM:
        return column(*self.figures, self.stats)

    def _draw(self) -> None:
        rows = self.sample()
        if rows is None:
            return
        rollover = rollover_for(self.window, self.fps)
        for sensor, source, p, row in zip(
            self.sensors, self.sources, self.figures, rows, strict=True
        ):
            source.stream({c: [row[c]] for c in source.data}, rollover=rollover)
            title = sensor_title(sensor)
            if p.title.text != title:
                p.title.text = title
        if self._stats_throttle.ready():
            self.stats.text = self._stats_html()

    def _sensor_figure(self, sensor, source, time_range, width, height, y_max) -> figure:
        p = time_series_figure(
            sensor_title(sensor),
            "Distance [m]",
            width,
            height,
            x_range=time_range,
            y_range=Range1d(0, y_max),
        )
        for style in ESTIMATES:
            p.line(
                "t",
                style.key,
                source=source,
                line_width=2,
                color=style.color,
                legend_label=f"Expected ({style.label})",
            )
        readings = p.scatter(
            "t", "measured", source=source, size=6, color=MEASURED_COLOR, legend_label="Measured"
        )
        tooltips = [("t", "@t{0.0} s"), ("Measured", "@measured{0.000} m")]
        tooltips += [(style.label, f"@{style.key}{{0.000}} m") for style in ESTIMATES]
        p.add_tools(HoverTool(renderers=[readings], tooltips=tooltips))
        return finish_figure(p)

    def _stats_html(self) -> str:
        lines = []
        for sensor, source in zip(self.sensors, self.sources, strict=True):
            measured = np.asarray(source.data["measured"], dtype=float)
            echoed = measured < sensor.config.max_range - NO_ECHO_TOLERANCE_M
            gaps = []
            for style in ESTIMATES:
                expected = np.asarray(source.data[style.key], dtype=float)
                gap = median_absolute_gap(measured[echoed], expected[echoed])
                gaps.append(f"{dot(style.color)} {style.label} {gap * 100:.1f} cm")
            lines.append(f"<b>{sensor.sensor_id.name}</b>: " + ", ".join(gaps))
        return (
            f"<span style='color:{MUTED_COLOR}'>Median gap between measured and expected distance "
            f"over the last {self.window:g} s:</span><br>" + "<br>".join(lines)
        )


def sensor_facing(sensor: UltrasonicSensor) -> str:
    degrees = round(math.degrees(sensor.config.angle))
    return FACING_BY_DEGREES.get(degrees, f"{degrees}°")


def sensor_title(sensor: UltrasonicSensor) -> str:
    source = "emulated from LiDAR" if sensor.emulated else "hardware"
    return f"{sensor.sensor_id.name} ({sensor_facing(sensor)}), {source}"


def plot_trajectories(df: pd.DataFrame, title: str = "Trajectories", size: int = 420, truth=None):
    p = top_view_figure(title, size)
    for style in ESTIMATES:
        x, y = to_plot(df[f"{style.column_prefix}_x"], df[f"{style.column_prefix}_y"])
        p.line(x, y, color=style.color, line_width=2, legend_label=style.label)
        finite = np.isfinite(x) & np.isfinite(y)
        if finite.any():
            p.scatter(
                [x[finite][-1]],
                [y[finite][-1]],
                color=style.color,
                size=9,
                legend_label=style.label,
            )
    if truth is not None:
        x, y = to_plot(*truth)
        p.line(
            x, y, color=MEASURED_COLOR, line_dash="dashed", line_width=2, legend_label="Reference"
        )
    finish_figure(p)
    show(p)
    return p


def plot_series(
    t,
    series: dict[str, object],
    title: str = "",
    y_label: str = "",
    dots: Sequence[str] = (),
    band: tuple[object, object, str] | None = None,
    width: int = 820,
    height: int = 280,
):
    p = time_series_figure(title, y_label, width, height, tools=f"{PLOT_TOOLS},hover")
    t = np.asarray(t, dtype=float)
    if band is not None:
        low, high, label = band
        p.varea(
            x=t,
            y1=np.asarray(low, dtype=float),
            y2=np.asarray(high, dtype=float),
            fill_color=KALMAN.color,
            fill_alpha=0.18,
            legend_label=label,
        )
    palette = SeriesPalette()
    for label, values in series.items():
        color = palette.color_for(label, measured=label in dots)
        y = np.asarray(values, dtype=float)
        if label in dots:
            p.scatter(t, y, size=5, color=color, legend_label=label)
        else:
            p.line(t, y, line_width=2, color=color, legend_label=label)
    finish_figure(p)
    show(p)
    return p
