# omni_drive.ui

```python
from omni_drive.ui import (
    LidarView, LidarViewConfig, LivePlot, PlannerView, UltrasonicView,
    median_absolute_gap, plot_series, plot_trajectories, rmse,
)
```

The content of each view is explained in [Visualizing](../guides/visualizing.md).

## Live views

`LidarView`, `UltrasonicView`, `LivePlot` and `PlannerView` share these methods:

| Method | Description |
| --- | --- |
| `show() -> Self` | displays the view under the cell and starts redrawing it `fps` times a second; returns the view. Stops any other running view of the same class. Raises `RuntimeError` when there is no running event loop, that is, outside a notebook |
| `stop() -> None` | stops redrawing |
| `running: bool` | whether the view is redrawing |

### LidarView

```python
LidarView(
    robot=None,
    fps: int = 10,
    config: LidarViewConfig | None = None,
    localizer: Localizer | None = None,
)
```

Top-down view of the LiDAR map and scan, with the three pose estimates drawn as robot
markers and the Kalman uncertainty as an ellipse. The view initially covers `view_range` meters around
the start and expands to fit the map. After a pan or zoom, the view remains fixed where the user
placed it; the reset tool makes it follow the map again.

| Parameter | Description |
| --- | --- |
| `robot` | if given (and `localizer` is not), the view uses `Localizer.for_robot(robot)` |
| `fps` | redraws per second |
| `config` | appearance of the view, described below |
| `localizer` | the localizer to draw; default `Localizer.default()` |

| Member | Description |
| --- | --- |
| `reset_pose() -> None` | resets the localizer (map and estimates) and clears the trails |
| `localizer: Localizer` | the localizer drawn by the view |
| `slam: LidarSlam` | `localizer.slam` |
| `plot` | the Bokeh figure |

### LidarViewConfig

```python
LidarViewConfig(view_range: float = 1.0, robot_radius: float = 0.17, sigma: float = 2.0)
```

| Field | Description |
| --- | --- |
| `view_range` | half-width of the view at the start, meters |
| `robot_radius` | radius of the robot markers, meters; default `hardware.ROBOT_RADIUS_M` |
| `sigma` | size of the Kalman ellipse in standard deviations |

Range rings are drawn every 0.5 m, out to the edge of the view.

### UltrasonicView

```python
UltrasonicView(
    robot=None,
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
odometry, LiDAR and Kalman poses as lines. Below the panels, the view shows the median absolute gap
between the readings and each line over the visible window, excluding readings at maximum
range. The panel titles indicate whether each sensor's readings come from the hardware or are emulated
from the LiDAR.

| Parameter | Description |
| --- | --- |
| `robot`, `localizer` | as for `LidarView` |
| `sensors` | sensors to show; default all three of `robot.ultrasonic_sensors` |
| `window` | seconds of history shown |
| `fps` | redraws per second |
| `width`, `height` | size of each panel, pixels |
| `y_max` | top of the distance axis, meters |
| `stats_refresh_s` | shortest time between updates of the gap line, seconds |

| Member | Description |
| --- | --- |
| `data: pandas.DataFrame` | every sample: `t`, `sensor`, `measured`, `odometry`, `lidar`, `kalman` (meters) |
| `sample() -> list[dict[str, object]] | None` | takes one sample per sensor if the localizer has a new estimate; called by the live loop |

### LivePlot

```python
LivePlot(
    title: str = "",
    y_label: str = "",
    window: float = 20.0,
    fps: float = 10,
    y_range: tuple[float, float] | None = None,
    width: int = 820,
    height: int = 260,
)
```

A live time-series plot of user-computed values.

| Parameter | Description |
| --- | --- |
| `title`, `y_label` | plot title and y-axis label |
| `window` | seconds of history shown |
| `fps` | samples and redraws per second |
| `y_range` | fixed y-axis range; default automatic |
| `width`, `height` | plot size, pixels |

| Member | Description |
| --- | --- |
| `add(name: str, fn: Callable[[], object], color: str | None = None, kind: Literal["line", "dots"] = "line") -> LivePlot` | adds a channel. `fn` takes no arguments and returns a number, a `Distance` (plotted in meters) or `None` (a gap). Returns the plot, so calls can be chained. Raises `ValueError` for a duplicate name, the name `"t"`, or another `kind` |
| `data: pandas.DataFrame` | every sample so far: `t` and one column per channel |
| `sample() -> dict[str, float]` | reads every channel once and records the row; called by the live loop |
| `clear() -> None` | deletes all samples |
| `figure` | the Bokeh figure |

```python
plot = LivePlot("Heading", y_label="yaw [rad]")
plot.add("odometry", lambda: robot.yaw).add("gyroscope", lambda: robot.imu_yaw)
plot.show()
```

## Offline plots

Both functions draw with `bokeh.plotting.show` and return the figure. Call
`bokeh.io.output_notebook()` once before using them in a notebook.

### plot_trajectories

```python
plot_trajectories(df: pd.DataFrame, title: str = "Trajectories", size: int = 420, truth=None)
```

Top view of the three paths in a `Localizer.dataframe()`, with a dot at each final position.
`truth`, if given, is a reference path `(xs, ys)` drawn dashed.

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

## Route planner

### PlannerView

```python
PlannerView(robot: OmniDrive | None = None, fps: float = 30)
```

A live view of the robot's odometry pose and the latest LiDAR points on a top-down canvas
with the same axes as `LidarView` and range rings every 0.5 m. Waypoints are placed by clicking
and moved by dragging; sliders set the smoothing and the speed of the next waypoint. The view
only draws the route; the robot does not follow it. `robot` defaults to `OmniDrive()`.

| Member | Description |
| --- | --- |
| `route() -> list[dict[str, float]]` | the route: the robot's position, then each waypoint, as `{"x": m, "y": m, "speed": %}` in the odometry frame; empty without waypoints |
| `robot` | the robot drawn by the view |

```python
planner = PlannerView().show()
...  # click waypoints
planner.route()
```

## Comparing series

```python
rmse(a, b) -> float
median_absolute_gap(a, b) -> float
```

Root mean square and median of `|a - b|` over the pairs where both values are finite (not
`nan` or `inf`). The result is `nan` when no pair remains. `UltrasonicView` uses `median_absolute_gap`.

```python
df = us_view.data
front = df[df["sensor"] == "US_1"]
median_absolute_gap(front["measured"], front["kalman"])
```
