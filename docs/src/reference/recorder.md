# omni_drive.recorder

Recorders sample user-supplied functions on every tick of a [`Clock`](clock.md) in a background
asyncio task. [`LivePlot`](live.md#liveplot) draws them.

```python
from omni_drive import Clock, PoseRecorder, SensorRecorder, Time

clock = Clock(period=Time.ms(100), duration=Time.s(20))
ir = SensorRecorder(
    clock,
    title="IR",
    front=lambda: robot.ir_front.distance.meters,
    left=lambda: robot.ir_left.distance.meters,
)
path = PoseRecorder(clock, odometry=lambda: (robot.pose.x.meters, robot.pose.y.meters))
```

## Recorder

```python
Recorder[DataT](clock: Clock | None = None, *, title: str = "", channels: dict[str, Any] | None = None)
```

Abstract base class of the recorders. `clock` defaults to a new `Clock()` (100 ms period, no end).

| Member | Description |
| --- | --- |
| `start() -> Self` | starts the recording task; returns `self`. Needs a running event loop. Raises `RuntimeError` if the recorder is already running |
| `stop() -> DataT` | cancels the recording task and returns `data` |
| `data: DataT` | the recorded data (abstract) |
| `record(t: Time) -> object` | samples every channel once and appends to `data` (abstract); called on each tick |
| `clock: Clock`, `title: str`, `channels: dict` | as passed in |

The task ends by itself when the clock reaches its `duration`. If a channel raises an
exception, the recording task ends. The exception is not raised in the cell; asyncio may
log it later as "Task exception was never retrieved".

## SensorRecorder

```python
SensorRecorder(
    clock: Clock | None = None,
    *,
    title: str = "",
    y_label: str = "",
    styles: dict[str, str] | None = None,
    **channels: Callable[[], float],
)
```

Records scalar time series. Each keyword argument in `channels` is a channel: a function with no
arguments that returns a number. The result is converted with `float()`, so return
`distance.meters`, not a `Distance`. Values that are not finite (`inf`, `nan`) are stored as
`nan`. Raises `ValueError` for a channel named `t`.

| Member | Description |
| --- | --- |
| `data: dict[str, list[float]]` | `"t"` (seconds, from the clock) and one list per channel |
| `record(t: Time) -> dict[str, float]` | samples once, appends, and returns the sample |
| `window: float` | seconds of history `LivePlot` shows: the clock's `duration` if a clock with a finite positive duration was passed, otherwise 30.0 |
| `y_label: str` | y-axis label for `LivePlot` |
| `styles: dict[str, str]` | how `LivePlot` draws each channel: `"line"` (default), `"scatter"`, `"line+scatter"` or `"dashed"` |

`"points"`, `"circle"` and `"point"` are accepted for `"scatter"`, `"line+points"` and `"both"`
for `"line+scatter"`, and `"dash"` for `"dashed"`. Any other value is drawn as a line.

```python
ir.start()
...  # drive
data = ir.stop()
data["front"][-1]
```

## PoseRecorder

```python
PoseRecorder(clock: Clock | None = None, *, title: str = "", **channels: Callable[[], tuple[float, float]])
```

Records 2D trajectories. Each channel returns an `(x, y)` pair, in meters by convention.
Coordinates that are not finite are stored as `nan`.

| Member | Description |
| --- | --- |
| `data: dict[str, list[Point2D]]` | one list of points per channel; there is no time column |
| `record(t: Time) -> dict[str, Point2D]` | samples once, appends, and returns the sample |

`DEFAULT_WINDOW_DURATION_S = 30.0` is a module constant.
