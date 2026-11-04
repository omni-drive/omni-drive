# Visualizing

The library has two kinds of plots. Both use [Bokeh](https://docs.bokeh.org/) and run in a
Jupyter notebook.

| Module | What it shows |
| --- | --- |
| `omni_drive.live` | `LivePlot`: live plots of values you record yourself with `SensorRecorder` and `PoseRecorder` |
| `omni_drive.ui` | ready-made views of the [localizer](localization.md), plots of a finished recording, and a route planner |

Call `output_notebook()` once at the top of the notebook:

```python
from bokeh.io import output_notebook

output_notebook(hide_banner=True)
```

## How live plots run

A live plot redraws from the notebook's event loop. It keeps redrawing while other code runs, as
long as that code waits with `await`, for example `await asyncio.sleep(...)`, `async for t in
clock` or `await robot.drive_for(...)`. A cell that blocks (`time.sleep`, a long computation)
freezes every plot until it ends.

Outside a notebook, `show()` raises `RuntimeError`, because no event loop is running.

## Record and plot your own values

You need three pieces:

- a `Clock`, which ticks at a fixed period,
- a recorder, which reads your values on every tick,
- a `LivePlot`, which draws the recorders.

```python
from omni_drive import Angle, Clock, DriveVector, OmniDrive, SensorRecorder, Time
from omni_drive.live import LivePlot

robot = OmniDrive()
clock = Clock(duration=Time.s(2))

rec = SensorRecorder(
    clock,
    title="Front sensors",
    y_label="distance [m]",
    styles={"ultrasonic": "line+scatter", "lidar": "dashed"},
    ultrasonic=lambda: robot.us_front.distance.meters,
    ir=lambda: robot.ir_front.distance.meters,
    lidar=lambda: robot.lidar.beam_front.meters,
)

async with LivePlot(rec), robot:
    async for t in clock:
        robot.set_vector(DriveVector(velocity=0.2, heading=Angle.deg(0)))
```

`LivePlot` is also imported as `from omni_drive import LivePlot`.

### Clock

`Clock(*, period=Time.ms(100), duration=Time.inf)` (keyword arguments only) ticks every `period` until `duration` has
passed. `async for t in clock` yields the elapsed time as a `Time`, starting at `t = 0`. Read
seconds with `t.seconds`.

One clock can drive several consumers at once. Above, the control loop and the recorder both
iterate over `clock`, so every sample matches one step of the loop. All consumers stop when the
duration ends. A consumer that falls behind gets the latest tick, not a queue of old ones.

`clock.stop()` ends the iteration for all consumers. You can iterate over a finished clock
again; it starts from `t = 0`.

### SensorRecorder

`SensorRecorder(clock=None, *, title="", y_label="", styles=None, **channels)` records numbers
over time.

- Each keyword argument is a channel: a function without arguments that returns a `float`. The
  argument name is the channel name.
- Return a number, not a `Distance`. Use `.meters`, for example
  `robot.us_front.distance.meters`.
- `inf` and `NaN` are recorded as `NaN` and show as a gap. A sensor with no reading returns
  `Distance.inf`, so it shows as a gap too.
- If a channel raises an exception, the recorder stops recording. Return `math.nan` instead.
- `t` is reserved for the time column and raises `ValueError`.

`rec.data` is a dictionary of lists: `"t"` (seconds) and one list per channel. Turn it into a
DataFrame with `pd.DataFrame(rec.data)`.

The plot shows a sliding time window: the clock's duration if it is finite, otherwise 30 s.

`styles` maps a channel name to how it is drawn:

| Style | Drawing |
| --- | --- |
| `"line"` (default) | solid line |
| `"scatter"`, `"points"`, `"point"`, `"circle"` | dots |
| `"line+scatter"`, `"line+points"`, `"both"` | thin line with dots |
| `"dashed"`, `"dash"` | dashed line |

Without `title` and `y_label`, the plot uses Polish defaults: "Wykres czujników" and
"Odległość [m]".

### PoseRecorder

`PoseRecorder(clock=None, *, title="", **channels)` records 2D paths. Each channel returns a
tuple `(x, y)` in meters.

```python
from omni_drive import PoseRecorder

robot.reset_pose()
rec = PoseRecorder(
    title="Odometry",
    odometry=lambda: (robot.pose.x.meters, robot.pose.y.meters),
)

async with LivePlot(rec), robot:
    await robot.drive_for(Time.s(2), DriveVector(velocity=0.3, heading=Angle.deg(45)))
```

`rec.data` maps each channel name to a list of `Point2D(x, y)`. Non-finite values become `NaN`.

The plot is a top view with the robot's start at the cross. x points up and y points left. The horizontal axis is labeled "Y [m]" but shows −y: a point 0.2 m to
the left of the start is drawn at −0.2. Without `title`, the plot is called "Trajektoria 2D".

### Without a clock

A recorder without a clock creates its own `Clock()`: every 100 ms, with no end. It records until
you stop it. `async with LivePlot(rec)` stops it when the block ends.

Every read of a channel calls your function once. A channel such as `lambda: kalman.state` for a
[`Kalman2d`](../concepts/kalman.md) runs one filter step per tick.

### LivePlot

`LivePlot(*recorders)` takes any number of `SensorRecorder` and `PoseRecorder` objects. Other
types raise `TypeError`. One recorder fills the width, two or three are placed in a row, four or
more in a grid of two columns.

| Use | Effect |
| --- | --- |
| `async with LivePlot(rec):` | shows the plot on entry and stops it when the block ends, also after an error |
| `plot = LivePlot(rec).show()` | shows the plot and starts every recorder that is not running yet |
| `plot.stop()` | stops redrawing and the recorders, and replaces the plot with a static copy |

The plot redraws 30 times a second
and shows at most the last 1000 samples per plot. `rec.data` keeps every sample.

`async with LivePlot(rec), robot:` combines the plot with the robot's context manager, which stops
the robot when the block ends.

To record without plotting, start and stop the recorder yourself. `stop()` returns the data:

```python
rec = SensorRecorder(front=lambda: robot.us_front.distance.meters).start()
...  # drive, with await
data = rec.stop()
```

## LiDAR map and pose estimates

```python
from omni_drive.ui import LidarView

view = LidarView().show()
```

The view shows:

- a top view with world x pointing up (the robot's front at the start) and world y pointing
  left, unlike a usual graph with y to the right,
- the occupancy map that the scan matcher builds: dark is occupied, white is free, gray is
  unknown,
- the latest scan, colored by distance, drawn at the LiDAR pose, with dotted range rings every
  0.5 m; hover over a point to see its distance,
- three robot markers, one per estimate, each with its trail: odometry (orange), LiDAR (blue) and
  Kalman filter (green); the line from the center of a marker shows its heading,
- a dashed ellipse around the Kalman marker at 2 standard deviations of its position (in 2D this
  contains about 86 % of the probability, not 95 %);
  it grows while the robot drives on odometry alone and shrinks when a LiDAR pose is accepted.

Below the plot, a table shows the three poses and:

- for odometry, its distance from the Kalman pose,
- for LiDAR, the status and the match score; the status is `waiting` (no scan yet), `tracking`,
  `lost` (the scan does not fit the map) or `rejected` (it fits the map, but the filter found it
  too far from its prediction),
- for the Kalman filter, its standard deviations,
- the time since the reset, the scan rate and how long one scan match takes.

The view first covers 1 m around the start and grows with the map. After you pan or zoom, it
keeps your extent. The reset tool in the toolbar makes it follow the map again. Click a legend
entry to hide it.

`view.reset_pose()` clears the map and the trails and resets the view's localizer.

`LidarView(robot=None, fps=10, config=None, localizer=None)`. Change the appearance with
`LidarViewConfig`:

| Field | Default | Meaning |
| --- | --- | --- |
| `view_range` | 1.0 | half-width of the initial view in meters |
| `robot_radius` | 0.17 | radius of the robot markers in meters |
| `sigma` | 2.0 | size of the Kalman ellipse in standard deviations |

```python
from omni_drive.ui import LidarView, LidarViewConfig

view = LidarView(config=LidarViewConfig(view_range=2.0, sigma=3.0)).show()
```

Only one view of each class runs at a time: showing a new `LidarView` stops the old one.
`view.stop()` stops redrawing. `view.running` tells whether it still redraws.

### Which localizer a view uses

`LidarView()` and `UltrasonicView()` use `Localizer.default()`, so they share one localizer.
`LidarView(robot)` uses `Localizer.for_robot(robot)`. To use your own localizer, pass
`localizer=`:

```python
from omni_drive import Localizer, PoseNoise
from omni_drive.ui import LidarView

loc = Localizer(noise=PoseNoise(per_meter=0.1)).start()
LidarView(localizer=loc).show()
```

## Ultrasonic sensors against the map

```python
from omni_drive.ui import UltrasonicView

us_view = UltrasonicView().show()
```

The view shows one panel per ultrasonic sensor. The gray dots are the sensor's readings. Each
line is the reading expected if the robot stood at one of the three estimated poses, computed
from the LiDAR map with `Localizer.expected_range` ([how](../concepts/ultrasonic.md)). The
closer a line follows the dots, the better that estimate.

Below the panels, the view shows the median absolute gap between the dots and each line, without
readings at maximum range. The median ignores occasional false echoes.

`UltrasonicView(robot=None, localizer=None, sensors=None, window=30.0, fps=10, width=820, height=200, y_max=1.2, stats_refresh_s=0.5)`:
`sensors` selects the sensors (all three by default), `window` is the visible time in seconds,
and `y_max` the top of the distance axis in meters.

`us_view.data` is every sample as a DataFrame with columns `t`, `sensor`, `measured`,
`odometry`, `lidar` and `kalman`.

## Offline plots

These functions draw a static plot from recorded data. They call `bokeh.plotting.show`, so call
`output_notebook()` first. Both return the Bokeh figure; assign it to `_` so Jupyter does not
print it.

### Trajectories from the localizer

`plot_trajectories(df, title="Trajectories", size=420, truth=None)` takes the DataFrame from
[`Localizer.dataframe()`](localization.md#record-a-run) and draws the three paths in the same
top view as `LidarView`.

```python
from omni_drive.ui import plot_trajectories

df = loc.dataframe()
_ = plot_trajectories(df)
```

`truth=(xs, ys)` adds a dashed reference path, for example one you measured.

### Time series

`plot_series(t, series, title="", y_label="", dots=(), band=None, width=820, height=280)` draws
several series against time. `series` maps a label to the values.

```python
from omni_drive.ui import plot_series

_ = plot_series(
    df["t"],
    {"odometry x": df["odom_x"], "LiDAR x": df["lidar_x"], "Kalman x": df["kf_x"]},
    y_label="x [m]",
    dots=["LiDAR x"],
    band=(df["kf_x"] - 2 * df["kf_std_x"], df["kf_x"] + 2 * df["kf_std_x"], "Kalman ±2σ"),
)
```

- Labels in `dots` are drawn as dots, the others as lines.
- `band=(low, high, label)` fills the area between two series.
- The first label that contains "odometr", "lidar" or "kalman" (any case) gets that estimate's
  color from `LidarView`. Other dotted series are gray.

It works with a `SensorRecorder` too:

```python
import pandas as pd

df = pd.DataFrame(rec.data)  # rec from the first example on this page (channels ir, lidar)
_ = plot_series(df["t"], {"IR": df["ir"], "LiDAR": df["lidar"]}, dots=["IR"])
```

### Error measures

| Function | Result |
| --- | --- |
| `rmse(a, b)` | root mean square of `a - b` |
| `median_absolute_gap(a, b)` | median of `abs(a - b)`, robust against outliers |

Both skip pairs where either value is `NaN` or infinite, and return `NaN` if no pair is left.

```python
from omni_drive.ui import rmse

rmse(df["ir"], df["lidar"])  # meters
```

## Route planner

```python
from omni_drive.ui import InteractiveMode, Steer

mode = Steer(robot).using(InteractiveMode())
```

This shows a canvas with the robot's odometry pose and the latest LiDAR points. Unlike
`LidarView`, x points right and y points up. Click to place a waypoint, and drag to move it. One
slider sets the smoothing of the whole route, the other the speed of the selected waypoint, in
percent; a new waypoint is selected and starts at 60 %.

The robot does not follow the route. The "Uruchom trasę" button only prints the route, and
"Wyczyść trasę" clears it. Read it with:

```python
mode.planner.get_payload()
# {"route": [{"x": ..., "y": ..., "speed": ...}, ...]}, meters and percent
```

The first entry is the robot's position. With no waypoints, the route is empty.

`Steer(robot).using(mode)` stops the mode that was active before. `mode.stop()` stops updating
the robot's position on the canvas.

See also the API pages [recorder](../reference/recorder.md), [clock](../reference/clock.md),
[live](../reference/live.md) and [ui](../reference/ui.md).
