from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

from bokeh.events import ButtonClick, Pan, PanEnd, PanStart
from bokeh.layouts import column, row
from bokeh.models import (
    Button,
    ColumnDataSource,
    CustomJS,
    Div,
    GlyphRenderer,
    LabelSet,
    LayoutDOM,
    PanTool,
    PointDrawTool,
    ResetTool,
    Slider,
    TapTool,
    WheelZoomTool,
)
from bokeh.plotting import figure

from omni_drive.hardware import ROBOT_RADIUS_M
from omni_drive.models import Pose
from omni_drive.robot import OmniDrive
from omni_drive.ui.live import LiveView
from omni_drive.ui.style import (
    HIGHLIGHT_COLOR,
    LIDAR_COLOR,
    MEASURED_COLOR,
    MUTED_COLOR,
    ODOMETRY_COLOR,
    RING_COLOR,
    SEGMENT_GUIDE_COLOR,
    SLOW_MEDIUM_FAST_COLORS,
    WAYPOINT_COLOR,
    finish_figure,
)
from omni_drive.ui.top_view import (
    draw_robot_marker,
    from_plot,
    robot_marker_data,
    scan_to_plot,
    top_view_figure,
)

DEFAULT_SPEED = 60.0
MIN_SPEED = 10.0
MAX_SPEED = 100.0
SPEED_STEP = 5.0
DEFAULT_SMOOTHING = 60.0
SMOOTHING_STEP = 5.0
CURVE_SAMPLES = 25
PLOT_SIZE = 560
PLOT_LIMIT = 2.0
CONTROLS_WIDTH = 240
RANGE_RINGS = (0.5, 1.0, 1.5, 2.0)
GRAB_RADIUS_PX = 14
EMPTY_ROUTE_STATS = "<b>Route</b><br>Waypoints: <b>0</b><br>Length: <b>0.00 m</b>"
SCRIPTS = Path(__file__).parent / "js"

Route = list[dict[str, float]]


@cache
def script(name: str) -> str:
    return (SCRIPTS / f"{name}.js").read_text(encoding="utf-8")


def _source(*columns: str) -> ColumnDataSource:
    return ColumnDataSource(data={column: [] for column in columns})


def _marker_at_origin() -> ColumnDataSource:
    return ColumnDataSource(data=robot_marker_data(Pose(), ROBOT_RADIUS_M))


@dataclass
class PlannerSources:
    waypoints: ColumnDataSource = field(default_factory=lambda: _source("x", "y"))
    speeds: ColumnDataSource = field(default_factory=lambda: _source("speed"))
    labels: ColumnDataSource = field(default_factory=lambda: _source("x", "y", "text"))
    trajectory: ColumnDataSource = field(default_factory=lambda: _source("xs", "ys", "colors"))
    highlight: ColumnDataSource = field(default_factory=lambda: _source("x", "y"))
    start: ColumnDataSource = field(default_factory=_marker_at_origin)
    robot: ColumnDataSource = field(default_factory=_marker_at_origin)
    scan: ColumnDataSource = field(default_factory=lambda: _source("x", "y"))


@dataclass
class PlannerControls:
    smoothing: Slider = field(
        default_factory=lambda: Slider(
            title=f"Trajectory smoothing [{DEFAULT_SMOOTHING:.0f}%]",
            value=DEFAULT_SMOOTHING,
            start=0.0,
            end=100.0,
            step=SMOOTHING_STEP,
            width=CONTROLS_WIDTH,
        )
    )
    speed: Slider = field(
        default_factory=lambda: Slider(
            title=f"Point speed [{DEFAULT_SPEED:.0f}%]",
            value=DEFAULT_SPEED,
            start=MIN_SPEED,
            end=MAX_SPEED,
            step=SPEED_STEP,
            width=CONTROLS_WIDTH,
        )
    )
    clear_button: Button = field(
        default_factory=lambda: Button(
            label="Clear route", button_type="warning", width=CONTROLS_WIDTH
        )
    )
    stats: Div = field(default_factory=lambda: Div(text=EMPTY_ROUTE_STATS, width=CONTROLS_WIDTH))


class PlannerView(LiveView):
    def __init__(self, robot: OmniDrive | None = None, fps: float = 30):
        super().__init__(fps)
        self.robot = robot or OmniDrive()
        self.sources = PlannerSources()
        self.controls = PlannerControls()
        self.plot = self._build_plot()

    def route(self) -> Route:
        waypoints = self.sources.waypoints.data
        if not len(waypoints["x"]):
            return []
        speeds = list(self.sources.speeds.data["speed"])
        speeds += [DEFAULT_SPEED] * (len(waypoints["x"]) - len(speeds))
        robot = self.sources.robot.data
        xs, ys = from_plot([robot["x"][0], *waypoints["x"]], [robot["y"][0], *waypoints["y"]])
        return [
            {
                "x": round(float(x), 3) + 0.0,
                "y": round(float(y), 3) + 0.0,
                "speed": round(float(speed), 1),
            }
            for x, y, speed in zip(xs, ys, [DEFAULT_SPEED, *speeds], strict=True)
        ]

    def _layout(self) -> LayoutDOM:
        controls = self.controls
        return row(
            self.plot,
            column(controls.smoothing, controls.speed, controls.clear_button, controls.stats),
        )

    def _draw(self) -> None:
        pose = self.robot.odometry
        if pose is None:
            return
        self.sources.robot.data = robot_marker_data(pose, ROBOT_RADIUS_M)
        scan = self.robot.lidar_scan
        if scan is not None:
            x, y = scan_to_plot(scan.points_in_meters(), pose)
            self.sources.scan.data = {"x": x, "y": y}

    def _build_plot(self) -> figure:
        p = top_view_figure(
            "Route planner",
            PLOT_SIZE,
            limit=PLOT_LIMIT,
            tools=[PanTool(), WheelZoomTool(), ResetTool(), TapTool()],
        )
        p.circle(
            x=0,
            y=0,
            radius=list(RANGE_RINGS),
            fill_color=None,
            line_color=RING_COLOR,
            line_dash="dotted",
        )
        p.scatter(
            x="x",
            y="y",
            source=self.sources.scan,
            size=2.5,
            color=LIDAR_COLOR,
            alpha=0.6,
            legend_label="Scan",
        )
        waypoints = self._draw_route(p)
        draw_robot_marker(
            p,
            self.sources.start,
            ROBOT_RADIUS_M,
            MUTED_COLOR,
            "Start",
            fill_alpha=0.0,
            line_dash="dotted",
        )
        draw_robot_marker(p, self.sources.robot, ROBOT_RADIUS_M, ODOMETRY_COLOR, "Robot (odometry)")
        self._bind_route_editing(p, waypoints)
        self._bind_waypoint_dragging(p)
        return finish_figure(p)

    def _draw_route(self, p: figure) -> GlyphRenderer:
        sources = self.sources
        p.line(
            x="x",
            y="y",
            source=sources.waypoints,
            line_width=1,
            line_color=SEGMENT_GUIDE_COLOR,
            line_dash="dashed",
        )
        p.multi_line(
            xs="xs",
            ys="ys",
            line_color="colors",
            line_width=4,
            source=sources.trajectory,
            legend_label="Trajectory",
        )
        p.scatter(
            x="x",
            y="y",
            source=sources.highlight,
            size=22,
            fill_color=None,
            line_color=HIGHLIGHT_COLOR,
            line_width=2.5,
            line_dash="dashed",
        )
        waypoints = p.scatter(
            x="x",
            y="y",
            source=sources.waypoints,
            size=12,
            color=WAYPOINT_COLOR,
            alpha=0.9,
            legend_label="Waypoints",
        )
        waypoints.nonselection_glyph.fill_alpha = 0.4
        waypoints.nonselection_glyph.line_alpha = 0.4
        p.add_layout(
            LabelSet(
                x="x",
                y="y",
                text="text",
                source=sources.labels,
                x_offset=8,
                y_offset=8,
                text_font_size="9pt",
                text_font_style="bold",
                text_color=MEASURED_COLOR,
                background_fill_color="white",
                background_fill_alpha=0.75,
                border_line_color=RING_COLOR,
                border_line_alpha=0.6,
                border_line_width=1,
            )
        )
        return waypoints

    def _bind_route_editing(self, p: figure, waypoints: GlyphRenderer) -> None:
        sources, controls = self.sources, self.controls
        draw_tool = PointDrawTool(renderers=[waypoints], add=True, drag=False)
        p.add_tools(draw_tool)
        p.toolbar.active_tap = draw_tool
        route_args = dict(
            waypoints=sources.waypoints,
            speeds=sources.speeds,
            labels=sources.labels,
            trajectory=sources.trajectory,
            highlight=sources.highlight,
            speed_slider=controls.speed,
            stats=controls.stats,
            empty_stats=EMPTY_ROUTE_STATS,
        )
        redraw = CustomJS(
            args=route_args
            | dict(
                robot=sources.robot,
                smoothing_slider=controls.smoothing,
                default_speed=DEFAULT_SPEED,
                min_speed=MIN_SPEED,
                max_speed=MAX_SPEED,
                samples=CURVE_SAMPLES,
                speed_colors=list(SLOW_MEDIUM_FAST_COLORS),
            ),
            code=script("update_trajectory"),
        )
        sources.waypoints.js_on_change("data", redraw)
        sources.waypoints.selected.js_on_change("indices", redraw)
        sources.robot.js_on_change("data", redraw)
        controls.smoothing.js_on_change("value", redraw)
        controls.speed.js_on_change("value", redraw)
        clear = CustomJS(args=route_args, code=script("clear_trajectory"))
        controls.clear_button.js_on_event(ButtonClick, clear)

    def _bind_waypoint_dragging(self, p: figure) -> None:
        waypoints = self.sources.waypoints
        start = CustomJS(
            args=dict(
                waypoints=waypoints,
                x_range=p.x_range,
                y_range=p.y_range,
                plot_size=PLOT_SIZE,
                grab_radius_px=GRAB_RADIUS_PX,
            ),
            code=script("drag_start"),
        )
        move = CustomJS(args=dict(waypoints=waypoints), code=script("drag_move"))
        end = CustomJS(args=dict(waypoints=waypoints), code="delete waypoints._drag;")
        p.js_on_event(PanStart, start)
        p.js_on_event(Pan, move)
        p.js_on_event(PanEnd, end)
