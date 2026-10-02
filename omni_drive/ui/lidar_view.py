import math
from dataclasses import dataclass

import numpy as np
from bokeh.events import MouseWheel, PanStart, Pinch, Reset
from bokeh.layouts import column
from bokeh.models import (
    ColorBar,
    ColumnDataSource,
    Div,
    HoverTool,
    LayoutDOM,
    LinearColorMapper,
    PanTool,
    ResetTool,
    WheelZoomTool,
)

from omni_drive.hardware import HOKUYO_MAX_RANGE_M, ROBOT_RADIUS_M
from omni_drive.localization import Estimate, Localizer
from omni_drive.models import Pose
from omni_drive.robot import OmniDrive
from omni_drive.ui.live import LiveView, Throttle
from omni_drive.ui.style import (
    ESTIMATES,
    KALMAN,
    LIDAR,
    MUTED_COLOR,
    NEAR_TO_FAR_PALETTE,
    ODOMETRY,
    RING_COLOR,
    WARNING_COLOR,
    dot,
    finish_figure,
    occupancy_palette,
)
from omni_drive.ui.top_view import (
    covariance_to_plot,
    draw_robot_marker,
    empty_marker_data,
    pose_to_plot,
    robot_marker_data,
    scan_to_plot,
    to_plot,
    top_view_figure,
)

STATS_EXTRA_WIDTH = 300
PLOT_SIZE = 560
POINT_SIZE = 4.0
RING_STEP = 0.5
VIEW_MARGIN = 0.25
TRAIL_LENGTH = 2000
TRAIL_MIN_STEP = 0.01
MAP_REFRESH_S = 0.5
STATS_REFRESH_S = 0.5


@dataclass(frozen=True)
class LidarViewConfig:
    view_range: float = 1.0
    robot_radius: float = ROBOT_RADIUS_M
    sigma: float = 2.0


class LidarView(LiveView):
    def __init__(
        self,
        robot: OmniDrive | None = None,
        fps: int = 10,
        config: LidarViewConfig | None = None,
        localizer: Localizer | None = None,
    ):
        super().__init__(fps)
        self.localizer = localizer or (
            Localizer.for_robot(robot) if robot is not None else Localizer.default()
        )
        self.config = config or LidarViewConfig()
        self._drawn_scan_seq = -1
        self._drawn_map_version = -1
        self._map_throttle = Throttle(MAP_REFRESH_S)
        self._stats_throttle = Throttle(STATS_REFRESH_S)
        self._trail_ends: dict[str, tuple[float, float]] = {}
        self._view_half_width = self.config.view_range
        self._follows_map = True

        self.scan_src = ColumnDataSource(data={"x": [], "y": [], "dist": []})
        self.rings_src = ColumnDataSource(data={"x": [], "y": [], "radius": []})
        self.map_src = ColumnDataSource(data=self._empty_map())
        self.ellipse_src = ColumnDataSource(data={"x": [], "y": [], "w": [], "h": [], "angle": []})
        self.marker_src = {s.key: ColumnDataSource(data=empty_marker_data()) for s in ESTIMATES}
        self.trail_src = {s.key: ColumnDataSource(data={"x": [], "y": []}) for s in ESTIMATES}
        self.stats = Div(
            text="Waiting for odometry and LiDAR data…",
            width=PLOT_SIZE + STATS_EXTRA_WIDTH,
        )
        self.plot = self._build_plot()

    @property
    def slam(self):
        return self.localizer.slam

    @property
    def color_range(self) -> float:
        return min(HOKUYO_MAX_RANGE_M, max(2 * self._view_half_width, 1.0))

    def reset_pose(self) -> None:
        self.localizer.reset()
        self._trail_ends.clear()
        self._drawn_map_version = -1
        for source in self.trail_src.values():
            source.data = {"x": [], "y": []}
        self.map_src.data = self._empty_map()

    def _layout(self) -> LayoutDOM:
        return column(self.plot, self.stats)

    def _build_plot(self):
        config = self.config
        p = top_view_figure(
            "Pose estimates: odometry, LiDAR and Kalman filter",
            PLOT_SIZE,
            limit=config.view_range,
            tools=[PanTool(), WheelZoomTool(), ResetTool()],
        )
        for gesture in (PanStart, MouseWheel, Pinch):
            p.on_event(gesture, self._stop_following_map)
        p.on_event(Reset, self._follow_map_again)
        p.image(
            image="image",
            x="x",
            y="y",
            dw="dw",
            dh="dh",
            source=self.map_src,
            color_mapper=LinearColorMapper(palette=occupancy_palette(), low=0, high=255),
        )
        p.circle(
            x="x",
            y="y",
            radius="radius",
            source=self.rings_src,
            fill_color=None,
            line_color=RING_COLOR,
            line_dash="dotted",
            legend_label=f"Range rings ({RING_STEP:g} m)",
        )
        self._distance_colors = LinearColorMapper(
            palette=list(NEAR_TO_FAR_PALETTE), low=0.0, high=self.color_range
        )
        distance = self._distance_colors
        scan = p.scatter(
            x="x",
            y="y",
            source=self.scan_src,
            size=POINT_SIZE,
            fill_color={"field": "dist", "transform": distance},
            line_color=None,
            fill_alpha=0.85,
            legend_label="Scan",
        )
        p.ellipse(
            x="x",
            y="y",
            width="w",
            height="h",
            angle="angle",
            source=self.ellipse_src,
            fill_color=KALMAN.color,
            fill_alpha=0.12,
            line_color=KALMAN.color,
            line_dash="dashed",
            legend_label=f"Kalman filter {config.sigma:g}σ",
        )
        for style in ESTIMATES:
            p.line(
                x="x",
                y="y",
                source=self.trail_src[style.key],
                line_color=style.color,
                line_width=2,
                line_alpha=0.9,
                legend_label=style.label,
            )
            draw_robot_marker(
                p, self.marker_src[style.key], config.robot_radius, style.color, style.label
            )
        p.add_tools(HoverTool(renderers=[scan], tooltips=[("Distance", "@dist{0.00} m")]))
        p.add_layout(
            ColorBar(color_mapper=distance, title="Distance [m]", width=10, padding=6), "right"
        )
        return finish_figure(p)

    def _draw(self) -> None:
        estimate = self.localizer.estimate
        if estimate is None:
            return
        poses = {
            ODOMETRY.key: estimate.odometry,
            LIDAR.key: estimate.lidar,
            KALMAN.key: estimate.kalman,
        }
        for key, pose in poses.items():
            self.marker_src[key].data = robot_marker_data(pose, self.config.robot_radius)
            if pose is not None:
                self._extend_trail(key, pose)
        self._draw_ellipse(estimate)
        self._draw_scan()
        if self.localizer.map_version != self._drawn_map_version and self._map_throttle.ready():
            self._draw_map()
        if self._stats_throttle.ready():
            self.stats.text = self._stats_html(estimate)

    def _extend_trail(self, key: str, pose: Pose) -> None:
        point = pose_to_plot(pose)
        end = self._trail_ends.get(key)
        if end is not None and math.dist(end, point) < TRAIL_MIN_STEP:
            return
        self._trail_ends[key] = point
        self.trail_src[key].stream({"x": [point[0]], "y": [point[1]]}, rollover=TRAIL_LENGTH)

    def _draw_ellipse(self, estimate: Estimate) -> None:
        variances, axes = np.linalg.eigh(covariance_to_plot(estimate.kalman_cov[:2, :2]))
        minor_std, major_std = np.sqrt(np.maximum(variances, 0.0))
        diameter_per_std = 2 * self.config.sigma
        x, y = pose_to_plot(estimate.kalman)
        self.ellipse_src.data = {
            "x": [x],
            "y": [y],
            "w": [diameter_per_std * major_std],
            "h": [diameter_per_std * minor_std],
            "angle": [math.atan2(axes[1, 1], axes[0, 1])],
        }

    def _draw_scan(self) -> None:
        frame = self.localizer.scan
        if frame is None or frame.seq == self._drawn_scan_seq:
            return
        self._drawn_scan_seq = frame.seq
        x, y = scan_to_plot(frame.points, frame.pose)
        self.scan_src.data = {
            "x": x.astype(np.float32),
            "y": y.astype(np.float32),
            "dist": np.hypot(frame.points[:, 0], frame.points[:, 1]).astype(np.float32),
        }
        cx, cy = pose_to_plot(frame.pose)
        radii = self._ring_radii()
        self.rings_src.data = {"x": [cx] * len(radii), "y": [cy] * len(radii), "radius": radii}

    def _draw_map(self) -> None:
        snapshot = self.localizer.map_snapshot()
        if snapshot is None:
            return
        self._drawn_map_version = snapshot.version
        rows, cols = snapshot.probability.shape
        height, width = rows * snapshot.resolution, cols * snapshot.resolution
        left, bottom = to_plot(snapshot.x_min, snapshot.y_min + width)
        left, bottom = float(left), float(bottom)
        self.map_src.data = {
            "image": [np.round(snapshot.probability[:, ::-1] * 255).astype(np.uint8)],
            "x": [left],
            "y": [bottom],
            "dw": [width],
            "dh": [height],
        }
        if self._follows_map:
            self._fit_view(left, left + width, bottom, bottom + height)

    def _fit_view(self, left: float, right: float, bottom: float, top: float) -> None:
        x_range, y_range = self.plot.x_range, self.plot.y_range
        inside = (
            left >= x_range.start
            and right <= x_range.end
            and bottom >= y_range.start
            and top <= y_range.end
        )
        if inside:
            return
        margin = VIEW_MARGIN
        half_width = max(
            self._view_half_width, (right - left) / 2 + margin, (top - bottom) / 2 + margin
        )
        center_x, center_y = (left + right) / 2, (bottom + top) / 2
        self._view_half_width = half_width
        for plot_range, center in ((x_range, center_x), (y_range, center_y)):
            plot_range.update(
                start=center - half_width,
                end=center + half_width,
                reset_start=center - half_width,
                reset_end=center + half_width,
            )
        self._distance_colors.high = self.color_range

    def _ring_radii(self) -> np.ndarray:
        count = int(self._view_half_width // RING_STEP)
        return np.arange(1, count + 1) * RING_STEP

    def _stop_following_map(self, _event=None) -> None:
        self._follows_map = False

    def _follow_map_again(self, _event=None) -> None:
        self._follows_map = True

    @staticmethod
    def _empty_map() -> dict[str, list]:
        return {
            "image": [np.zeros((1, 1), np.uint8)],
            "x": [0.0],
            "y": [0.0],
            "dw": [0.0],
            "dh": [0.0],
        }

    def _stats_html(self, estimate: Estimate) -> str:
        std_x, std_y, std_yaw = estimate.kalman_std
        odometry_gap = estimate.odometry.distance_to(estimate.kalman)
        rows = (
            _pose_row(
                ODOMETRY.label,
                ODOMETRY.color,
                estimate.odometry,
                f"{odometry_gap * 100:.1f} cm from Kalman filter",
            ),
            _pose_row(
                LIDAR.label,
                LIDAR.color,
                estimate.lidar,
                f"{_lidar_status(estimate)}, match {estimate.lidar_score:.0%}",
            ),
            _pose_row(
                KALMAN.label,
                KALMAN.color,
                estimate.kalman,
                f"σ {std_x * 100:.1f} / {std_y * 100:.1f} cm, {math.degrees(std_yaw):.1f}°",
            ),
        )
        footer = " &nbsp;|&nbsp; ".join(
            (
                f"t = {estimate.t:.1f} s",
                f"Scans {self.localizer.scan_hz:.1f} Hz",
                f"Scan matching {self.localizer.slam_ms:.0f} ms",
                "Map: dark = occupied, white = free, gray = unknown",
            )
        )
        return (
            f"<table style='border-spacing:10px 2px'>{''.join(rows)}</table>"
            f"<span style='color:{MUTED_COLOR}'>{footer}</span>"
        )


def _lidar_status(estimate: Estimate) -> str:
    if estimate.lidar is None:
        return "waiting"
    if not estimate.lidar_confident:
        return f"<span style='color:{WARNING_COLOR}'>lost</span>"
    if not estimate.lidar_accepted:
        return f"<span style='color:{WARNING_COLOR}'>rejected</span>"
    return "tracking"


def _pose_row(label: str, color: str, pose: Pose | None, note: str) -> str:
    if pose is None:
        cells = "<td colspan=3>—</td>"
    else:
        cells = (
            f"<td>x <b>{pose.x:+.3f}</b> m</td>"
            f"<td>y <b>{pose.y:+.3f}</b> m</td>"
            f"<td>yaw <b>{math.degrees(pose.yaw):+.1f}°</b></td>"
        )
    return f"<tr><td>{dot(color)}</td><td>{label}</td>{cells}<td>{note}</td></tr>"
