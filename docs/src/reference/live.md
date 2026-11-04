# omni_drive.live

Live Bokeh plots of [recorders](recorder.md) in a Jupyter notebook.

```python
from omni_drive.live import (
    DEFAULT_PLOT_HEIGHT, LivePlot, LiveView, Plot, PosePlot, SensorPlot,
)
```

`LivePlot` is also exported from `omni_drive`. It is a different class from the `LivePlot` in
`omni_drive.ui.plots`, which is not documented here.

## LivePlot

```python
LivePlot(*recorders: Recorder)
```

Draws one figure per recorder: a [`SensorPlot`](#sensorplot) for each `SensorRecorder` and a
[`PosePlot`](#poseplot) for each `PoseRecorder`. Raises `TypeError` for any other recorder.
One figure fills the width; two or three are placed in a row; four or more in a grid of two
columns. The view redraws 30 times a second.

| Member | Description |
| --- | --- |
| `show() -> Self` | starts every recorder that is not running, displays the figures under the cell and starts redrawing. Raises `RuntimeError` without a running event loop |
| `stop() -> None` | stops redrawing, draws once more, replaces the live widget with a static copy (an HTML frame 400 px high), and stops every recorder |
| `recorders: tuple[Recorder, ...]` | as passed in |

`LivePlot` is an async context manager: entering calls `show()`, leaving calls `stop()`.

```python
ir = SensorRecorder(front=lambda: robot.ir_front.distance.meters)
path = PoseRecorder(odometry=lambda: (robot.pose.x.meters, robot.pose.y.meters))

async with LivePlot(ir, path):
    await robot.drive_for(Time.s(3), DriveVector(0.3, Angle.deg(0)))
```

## SensorPlot

```python
SensorPlot(
    recorder: SensorRecorder,
    height: int = DEFAULT_PLOT_HEIGHT,
    rollover: int = 1000,
    styles: dict[str, str] | None = None,
)
```

A time-series figure for a `SensorRecorder`, one series per channel. The x axis shows the last
`recorder.window` seconds. `styles` overrides `recorder.styles` per channel (same values). The
figure keeps at most `rollover` samples. Clicking a legend entry hides the series. Without
`title` and `y_label` on the recorder, the figure uses the Polish defaults
`"Wykres czujników"` and `"Odległość [m]"`.

| Member | Description |
| --- | --- |
| `update() -> None` | adds the samples recorded since the last call |
| `figure` | the Bokeh figure |
| `recorder`, `rollover` | as passed in |

## PosePlot

```python
PosePlot(recorder: PoseRecorder, height: int = DEFAULT_PLOT_HEIGHT, rollover: int = 1000)
```

A top-down trajectory figure for a `PoseRecorder`: one line per channel, a dot at its latest
point and a cross at `(0, 0)`. Forward (+x) points up; the horizontal axis shows −y, so left
(+y) is on the left. The figure keeps
at most `rollover` points per channel. Without a `title`, the figure uses the Polish default
`"Trajektoria 2D"`.

| Member | Description |
| --- | --- |
| `update() -> None` | adds the points recorded since the last call |
| `figure` | the Bokeh figure |
| `recorder`, `rollover` | as passed in |

## LiveView

```python
LiveView(fps: float = 30.0)
```

Abstract base class of `LivePlot`: an async loop that redraws a Bokeh layout in the notebook
`fps` times a second. Subclasses implement `_layout()` and `_draw()`. Errors raised while
drawing are printed and the loop continues.

| Member | Description |
| --- | --- |
| `show() -> Self` | displays the layout and starts redrawing. Raises `RuntimeError` without a running event loop |
| `stop() -> None` | stops redrawing and replaces the widget with a static copy |
| `fps: float` | redraws per second |

`LiveView` is an async context manager: entering calls `show()`, leaving calls `stop()`.

## Plot

```python
class Plot(Protocol):
    figure: figure
    def update(self) -> None: ...
```

The protocol `SensorPlot` and `PosePlot` implement.

## DEFAULT_PLOT_HEIGHT

`DEFAULT_PLOT_HEIGHT = 360`, the default figure height in pixels.
