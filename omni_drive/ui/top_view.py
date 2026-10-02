import math

import numpy as np
from bokeh.models import ColumnDataSource, CustomJSTickFormatter, GlyphRenderer, Legend
from bokeh.plotting import figure

from omni_drive.lidar_slam import to_world
from omni_drive.models import Pose
from omni_drive.ui.style import finish_figure

WORLD_TO_PLOT = np.array([[0.0, -1.0], [1.0, 0.0]])
HEADING_LENGTH_PER_RADIUS = 1.8
NEGATED_TICK_JS = """
const step = ticks.length > 1 ? Math.abs(ticks[1] - ticks[0]) : 1;
const digits = Math.max(0, Math.ceil(-Math.log10(step)));
return (-tick + 0).toFixed(digits);
"""
MARKER_COLUMNS = ("x", "y", "hx", "hy")


def to_plot(x, y):
    return -np.asarray(y, dtype=float), np.asarray(x, dtype=float)


def from_plot(u, v):
    return np.asarray(v, dtype=float), -np.asarray(u, dtype=float)


def pose_to_plot(pose: Pose) -> tuple[float, float]:
    return -pose.y, pose.x


def covariance_to_plot(position_covariance: np.ndarray) -> np.ndarray:
    return WORLD_TO_PLOT @ position_covariance @ WORLD_TO_PLOT.T


def scan_to_plot(points: np.ndarray, pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    world = to_world(points, pose.x, pose.y, pose.yaw)
    return to_plot(world[:, 0], world[:, 1])


def top_view_figure(title: str, size: int, limit: float | None = None, **kwargs) -> figure:
    if limit is not None:
        kwargs |= {"x_range": (-limit, limit), "y_range": (-limit, limit)}
    p = figure(
        title=title,
        frame_width=size,
        frame_height=size,
        match_aspect=True,
        x_axis_label="y [m] (left +)",
        y_axis_label="x [m] (forward +)",
        **kwargs,
    )
    p.xaxis.formatter = CustomJSTickFormatter(code=NEGATED_TICK_JS)
    p.add_layout(Legend(location="top_left"), "right")
    return finish_figure(p)


def empty_marker_data() -> dict[str, list[float]]:
    return {column: [] for column in MARKER_COLUMNS}


def robot_marker_data(pose: Pose | None, radius: float) -> dict[str, list[float]]:
    if pose is None:
        return empty_marker_data()
    length = radius * HEADING_LENGTH_PER_RADIUS
    tip = Pose(pose.x + length * math.cos(pose.yaw), pose.y + length * math.sin(pose.yaw))
    (x, y), (hx, hy) = pose_to_plot(pose), pose_to_plot(tip)
    return {"x": [x], "y": [y], "hx": [hx], "hy": [hy]}


def draw_robot_marker(
    p: figure,
    source: ColumnDataSource,
    radius: float,
    color: str,
    label: str | None = None,
    fill_alpha: float = 0.18,
    line_dash: str = "solid",
) -> GlyphRenderer:
    legend = {"legend_label": label} if label else {}
    body = p.circle(
        x="x",
        y="y",
        radius=radius,
        source=source,
        fill_color=color,
        fill_alpha=fill_alpha,
        line_color=color,
        line_width=2,
        line_dash=line_dash,
        **legend,
    )
    p.segment(
        x0="x",
        y0="y",
        x1="hx",
        y1="hy",
        source=source,
        line_color=color,
        line_width=3,
        line_dash=line_dash,
        **legend,
    )
    return body
