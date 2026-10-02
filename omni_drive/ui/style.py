from itertools import cycle
from typing import NamedTuple

import numpy as np
from bokeh.models import Legend
from bokeh.plotting import figure

ODOMETRY_COLOR = "#eb6834"
LIDAR_COLOR = "#2a78d6"
KALMAN_COLOR = "#1baf7a"
MEASURED_COLOR = "#52514e"
MUTED_COLOR = "#777777"
RING_COLOR = "#c8c8c8"
WARNING_COLOR = "#e34948"
WAYPOINT_COLOR = "#1f77b4"
HIGHLIGHT_COLOR = "#ff7f0e"
SEGMENT_GUIDE_COLOR = "#b0b0b0"
SLOW_MEDIUM_FAST_COLORS = ("#4682b4", "#2ecc71", "#e74c3c")
EXTRA_COLORS = ("#8e5bd0", "#d1477a", "#c49a00", "#16a3b8", "#8c564b", "#7f7f7f")
NEAR_TO_FAR_PALETTE = (
    "#0d366b",
    "#104281",
    "#184f95",
    "#1c5cab",
    "#256abf",
    "#2a78d6",
    "#3987e5",
    "#5598e7",
    "#6da7ec",
    "#86b6ef",
)
FREE_RGB, UNKNOWN_RGB, OCCUPIED_RGB = (255, 255, 255), (232, 232, 232), (40, 40, 40)


class EstimateStyle(NamedTuple):
    key: str
    label: str
    color: str
    column_prefix: str
    label_keyword: str


ODOMETRY = EstimateStyle("odometry", "Odometry", ODOMETRY_COLOR, "odom", "odometr")
LIDAR = EstimateStyle("lidar", "LiDAR", LIDAR_COLOR, "lidar", "lidar")
KALMAN = EstimateStyle("kalman", "Kalman filter", KALMAN_COLOR, "kf", "kalman")
ESTIMATES = (ODOMETRY, LIDAR, KALMAN)

PLOT_TOOLS = "pan,wheel_zoom,reset,save"


class SeriesPalette:
    def __init__(self):
        self._unclaimed = list(ESTIMATES)
        self._extra = cycle(EXTRA_COLORS)

    def color_for(self, label: str, measured: bool = False) -> str:
        words = label.lower()
        for style in self._unclaimed:
            if style.label_keyword in words:
                self._unclaimed.remove(style)
                return style.color
        return MEASURED_COLOR if measured else next(self._extra)


def time_series_figure(title: str, y_label: str, width: int, height: int, **kwargs) -> figure:
    p = figure(
        title=title,
        width=width,
        height=height,
        x_axis_label="t [s]",
        y_axis_label=y_label,
        tools=kwargs.pop("tools", PLOT_TOOLS),
        **kwargs,
    )
    p.add_layout(Legend(location="top_left"), "right")
    return finish_figure(p)


def finish_figure(p: figure) -> figure:
    p.grid.grid_line_alpha = 0.3
    if p.legend:
        p.legend.click_policy = "hide"
        p.legend.background_fill_alpha = 0.7
    return p


def dot(color: str) -> str:
    return f"<span style='color:{color}'>&#9679;</span>"


def occupancy_palette(steps: int = 256) -> list[str]:
    probability = np.linspace(0.0, 1.0, steps)
    channels = [
        np.interp(probability, (0.0, 0.5, 1.0), stops)
        for stops in zip(FREE_RGB, UNKNOWN_RGB, OCCUPIED_RGB, strict=True)
    ]
    rgb = np.round(np.column_stack(channels)).astype(int)
    return [f"#{r:02x}{g:02x}{b:02x}" for r, g, b in rgb]
