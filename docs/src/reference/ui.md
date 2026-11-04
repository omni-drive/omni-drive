# omni_drive.ui

```python
from omni_drive.ui import (
    InteractiveMode, LidarView, LidarViewConfig, Steer, UltrasonicView,
    median_absolute_gap, plot_series, plot_trajectories, rmse,
)
```

Notebook views built on Bokeh. For live plots of recorders, see [`omni_drive.live`](live.md).

## Live views

`LidarView` and `UltrasonicView` derive from `omni_drive.ui.live.LiveView` and share these
members:

| Member | Description |
| --- | --- |
| `show() -> Self` | displays the view under the cell and starts redrawing it `fps` times a second; returns the view. Stops any other running view of the same class. Raises `RuntimeError` when there is no running event loop, that is, outside a notebook |
| `stop() -> None` | stops redrawing; the last drawing stays |
| `running: bool` | whether the view is redrawing |
| `fps: float` | redraws per second |

An error raised while drawing is printed once, and the view keeps redrawing.

Both views read from a [`Localizer`](localization.md#localizer). Without `localizer` and
`robot`, they use `Localizer.default()`, which starts a localizer for `OmniDrive()` if none is
running.

### LidarView

```python
LidarView(
    robot: OmniDrive | None = None,
    fps: int = 10,
    config: LidarViewConfig | None = None,
    localizer: Localizer | None = None,
)
```

Top-down view of the LiDAR map and the latest scan (colored by distance), with the odometry,
LiDAR and Kalman estimates drawn as robot markers with trails and the Kalman position
uncertainty as an ellipse. Forward (+x) points up and left (+y) points left. The view initially
covers `view_range` meters around the start and expands to fit the map. After a pan or zoom, the
view stays where the user put it; the reset tool makes it follow the map again. Range rings are
drawn every 0.5 m around the LiDAR pose, out to the edge of the view. Below the plot, a table
shows each pose, the LiDAR status (waiting, tracking, rejected or lost) and match score, the
Kalman standard deviations, the scan rate and the matching time.

| Parameter | Description |
| --- | --- |
| `robot` | if given (and `localizer` is not), the view uses `Localizer.for_robot(robot)` |
| `fps` | redraws per second |
| `config` | appearance of the view, described below |
| `localizer` | the localizer to draw; default `Localizer.default()` |

| Member | Description |
| --- | --- |
| `reset_pose() -> None` | resets the localizer (map and estimates) and clears the trails and the map image |
| `localizer: Localizer` | the localizer drawn by the view |
| `config: LidarViewConfig` | as passed in or created |
| `slam: LidarSlam` | `localizer.slam` |
| `plot` | the Bokeh figure |
| `stats` | the Bokeh `Div` with the table |

### LidarViewConfig

```python
LidarViewConfig(view_range: float = 1.0, robot_radius: float = ROBOT_RADIUS_M, sigma: float = 2.0)
```

The dataclass is frozen.

| Field | Description |
| --- | --- |
| `view_range` | half-width of the view at the start, meters |
| `robot_radius` | radius of the robot markers, meters; default `omni_drive.common.ROBOT_RADIUS_M` (0.17) |
| `sigma` | size of the Kalman ellipse in standard deviations |

### UltrasonicView

```python
UltrasonicView(
    robot: OmniDrive | None = None,
    localizer: Localizer | None = None,
    sensors: Sequence[UltrasonicSensor] | None = None,
    window: float = 30.0,
    fps: float = 10,
    width: int = 820,
    height: int = 200,
    y_max: float = 1.2,
    stats_refresh_s: float = 0.5,
)
```

One panel per sensor: the sensor's readings as dots, and the readings expected at the
odometry, LiDAR and Kalman poses (from `Localizer.expected_range`) as lines. Below the panels,
the view shows the median absolute gap between the readings and each line over the visible
window, excluding readings at or beyond `config.max_range`.

| Parameter | Description |
| --- | --- |
| `robot`, `localizer` | as for `LidarView` |
| `sensors` | sensors to show; default `robot.ultrasonic_sensors` (left, front, right) |
| `window` | seconds of history shown |
| `fps` | redraws per second |
| `width`, `height` | size of each panel, pixels |
| `y_max` | top of the distance axis, meters |
| `stats_refresh_s` | shortest time between updates of the gap line, seconds |

| Member | Description |
| --- | --- |
| `data: pandas.DataFrame` | every sample: `t`, `sensor` (`"LEFT"`, `"FRONT"`, `"RIGHT"`), `measured`, `odometry`, `lidar`, `kalman` (meters) |
| `sample() -> list[dict[str, object]] | None` | takes one sample per sensor if the localizer has a new estimate with a LiDAR pose, otherwise returns `None`; called by the live loop |
| `localizer`, `robot`, `sensors`, `window` | as passed in or derived |

## Steering modes

### Steer

```python
Steer(robot: OmniDrive)
```

| Member | Description |
| --- | --- |
| `using(mode: ControlMode) -> ControlMode` | stops the previously active mode (shared by all `Steer` objects) if it is a different one, then calls `mode.start(robot)` and returns `mode` |
| `robot` | as passed in |

A `ControlMode` is any object with `start(robot) -> None` and `stop() -> None`.

```python
mode = Steer(robot).using(InteractiveMode())
```

### InteractiveMode

```python
InteractiveMode(fps: int = 30)
```

A route planner canvas with the robot's live position. `start(robot)` displays the planner under
the cell and starts an async loop that, `fps` times a second, draws the robot at `robot.pose`
with a heading arrow and the latest LiDAR points around it. Waypoints are added by clicking and
moved by dragging; one slider sets the smoothing of the whole route, the other the speed of the selected waypoints. The
canvas has x to the right and y up, unlike `LidarView`.

The run button prints the route as
`[Trajectory Run] {"route": [...]}`.

| Member | Description |
| --- | --- |
| `start(robot: OmniDrive) -> None` | stops any other active `InteractiveMode`, shows the planner and starts the loop. Needs a running event loop |
| `stop() -> None` | stops the loop; the canvas stays |
| `planner` | the `TrajectoryPlanner` (below) |
| `robot: OmniDrive | None` | the robot passed to `start` |
| `fps` | as passed in |

`planner.get_payload() -> dict` returns `{"route": [...]}`: the robot's position, then each
waypoint, as `{"x": m, "y": m, "speed": %}` in the odometry frame. The route is empty without
waypoints. The default speed is 60 %. `planner.cfg` is the `PlannerConfig` with the canvas and
slider settings.

## Offline plots

Both functions draw with `bokeh.plotting.show` and return the figure. Call
`bokeh.io.output_notebook()` once before using them in a notebook.

### plot_trajectories

```python
plot_trajectories(df: pd.DataFrame, title: str = "Trajectories", size: int = 420, truth=None)
```

Top view of the odometry, LiDAR and Kalman paths in a `Localizer.dataframe()` (columns
`odom_x`, `odom_y`, `lidar_x`, `lidar_y`, `kf_x`, `kf_y`), with a dot at each final position.
`truth`, if given, is a reference path `(xs, ys)` drawn dashed. Same orientation as `LidarView`.

### plot_series

```python
plot_series(
    t,
    series: dict[str, object],
    title: str = "",
    y_label: str = "",
    dots: Sequence[str] = (),
    band: tuple[object, object, str] | None = None,
    width: int = 820,
    height: int = 280,
)
```

Plots each entry of `series` (legend label to values) against `t`. Labels listed in `dots` are
drawn as dots, the rest as lines. `band = (low, high, label)` shades the area between two
series. The first label containing "odometr", "lidar" or "kalman" (any case) gets that
estimate's color from the live views; other dotted series are gray.

## Comparing series

```python
rmse(a, b) -> float
median_absolute_gap(a, b) -> float
```

Root mean square and median of `|a - b|` over the pairs where both values are finite (not
`nan` or `inf`). The result is `nan` when no pair remains.

```python
df = loc.dataframe()
rmse(df["odom_x"], df["kf_x"])
```
