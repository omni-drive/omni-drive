# Visualizing

`omni_drive.ui` has live views that redraw while the robot runs, and helpers that plot a
recording afterwards. All of them use [Bokeh](https://docs.bokeh.org/).

## How live views run

A live view redraws from the notebook's event loop, about 10 times a second. It keeps
redrawing while other cells run, as long as they wait with `await asyncio.sleep(...)`. A cell
that blocks (`time.sleep`, a long computation) freezes every view until it ends.

Only one view of each class runs at a time, so running the cell again replaces the old view.
`view.stop()` stops redrawing. Outside a notebook, `show()` raises `RuntimeError`, because
no event loop is running.

## LiDAR map and pose estimates

```python
from omni_drive.ui import LidarView

view = LidarView().show()
```

The view contains:

- a map seen from above, with world x pointing up (the robot's front at the start) and world y
  pointing left, unlike a graph with y to the right,
- the occupancy map the scan matcher builds: dark is occupied, white is free, gray is unknown,
- the latest scan, colored by distance, drawn at the LiDAR pose, with dotted range rings
  every 0.5 m around it,
- three robot markers, one per pose estimate, each with its trail: odometry (orange),
  LiDAR (blue) and Kalman filter (green). The line from the center of a marker shows its
  heading, and the LiDAR marker appears with the first scan,
- a dashed ellipse around the Kalman marker, 2 standard deviations of its position. It grows
  while the robot drives on odometry alone and shrinks when a LiDAR pose is accepted.

Below the plot, a table shows the three poses and:

- the LiDAR status: `tracking`, `lost` (the scan does not fit the map) or `rejected` (it fits
  the map, but the filter found it too far from its prediction), and the match score;
- the Kalman standard deviations;
- the scan rate and how long scan matching takes.

The view initially covers 1 m around the start position and grows with the map. After a pan
or zoom it keeps the chosen extent, and the reset tool in the toolbar makes it follow the map
again. Clicking a legend entry hides it. `view.reset_pose()` clears the map and the trails and resets
the shared localizer.

The appearance is configured with `LidarViewConfig`:

```python
from omni_drive.ui import LidarView, LidarViewConfig

view = LidarView(config=LidarViewConfig(view_range=2.0, sigma=3.0)).show()
```

## Ultrasonic sensors against the map

```python
from omni_drive.ui import UltrasonicView

us_view = UltrasonicView().show()
```

The view shows one panel per ultrasonic sensor. The gray dots are the sensor's readings. Each
line is the reading expected if the robot stood at one of the three estimated poses, computed
from the LiDAR map ([how](../concepts/ultrasonic.md)). The more closely a line follows the
dots, the more accurate that estimate.

Below the panels, the view shows the median absolute difference between the dots and each
line, excluding readings at maximum range. The median is used because it is insensitive to
occasional false echoes.

Each panel title states whether the readings come from the sensor ("hardware") or are
[emulated from LiDAR](sensors.md#emulated-readings), and it follows the source while the
view runs. `UltrasonicView(y_max=2.0)` changes the top of the distance axis, 1.2 m by default.

`us_view.data` is every sample as a DataFrame with columns `t`, `sensor`, `measured`,
`odometry`, `lidar` and `kalman`.

`LidarView()` and `UltrasonicView()` use `Localizer.default()`, so they share one localizer.
`LidarView(robot)` does the same for the current robot (`Localizer.for_robot(robot)`). To pass
a different localizer, use `localizer=`:

```python
from omni_drive import Localizer, PoseNoise
from omni_drive.ui import LidarView, UltrasonicView

loc = Localizer(noise=PoseNoise(per_meter=0.1)).start()
LidarView(localizer=loc).show()
UltrasonicView(localizer=loc).show()
```

## Plot your own values live

```python
from omni_drive.ui import LivePlot

plot = LivePlot("Front sensor", y_label="distance [m]", window=30)
plot.add("ultrasonic", lambda: robot.us_1.read(), kind="dots")
plot.add("odometry x", lambda: robot.odometry.x)
plot.show()
```

Each channel is a function with no arguments. It can return a number, a `Distance` (plotted
in meters) or `None` (a gap). If it raises an exception, that sample is a gap too. The plot
shows the last `window` seconds. Channel names must be unique, and `t` is reserved.

`plot.data` holds every sample so far as a DataFrame, with a `t` column and one column per
channel. `plot.clear()` empties it. To record without drawing, omit `show()` and call
`plot.sample()` directly.

## Offline plots

These functions take recorded data, for example from `Localizer.dataframe()`, and draw a static
plot. They use `bokeh.plotting.show`, so call `output_notebook()` once beforehand.

```python
from bokeh.io import output_notebook

from omni_drive.ui import plot_series, plot_trajectories

output_notebook()
df = loc.dataframe()

_ = plot_trajectories(df)
_ = plot_series(
    df["t"],
    {"odometry x": df["odom_x"], "LiDAR x": df["lidar_x"], "Kalman x": df["kf_x"]},
    y_label="x [m]",
    dots=["LiDAR x"],
    band=(df["kf_x"] - 2 * df["kf_std_x"], df["kf_x"] + 2 * df["kf_std_x"], "Kalman ±2σ"),
)
```

Both return the Bokeh figure; assigning it to `_` prevents Jupyter from printing it.

- `plot_trajectories(df, truth=(xs, ys))` adds a dashed reference path, for example the
  simulator's `true_pose` recorded during the run.
- In `plot_series`, the first label containing "odometr", "lidar" or "kalman" (any case) gets
  that estimate's color from the live views. Other dotted series are gray.

## Route planner

```python
from omni_drive.ui import PlannerView

planner = PlannerView().show()
```

This shows the robot's odometry pose and the latest LiDAR points on a canvas with the same axes
as `LidarView`. Clicking places a waypoint, and dragging moves it. The sliders set the
smoothing and the speed of the next waypoint. The robot does not follow the route; the route
is read with:

```python
planner.route()  # [{"x": ..., "y": ..., "speed": ...}, ...], meters and percent
```

The first entry is the robot's position.
