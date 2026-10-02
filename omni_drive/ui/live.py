import asyncio
import time
from typing import ClassVar, Self

from bokeh.models import LayoutDOM
from IPython.display import display
from jupyter_bokeh.widgets import BokehModel


class Throttle:
    def __init__(self, interval_s: float):
        self.interval_s = interval_s
        self._last = -float("inf")

    def ready(self) -> bool:
        now = time.monotonic()
        if now - self._last < self.interval_s:
            return False
        self._last = now
        return True


class LiveView:
    _active: ClassVar[dict[type, "LiveView"]] = {}

    def __init__(self, fps: float = 10):
        self.fps = fps
        self._task: asyncio.Task | None = None
        self._last_error = ""

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    def show(self) -> Self:
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError as error:
            raise RuntimeError(
                f"{type(self).__name__}.show() needs a running event loop, e.g. a Jupyter notebook"
            ) from error
        previous = LiveView._active.get(type(self))
        if previous is not None:
            previous.stop()
        display(BokehModel(self._layout()))
        LiveView._active[type(self)] = self
        self._task = loop.create_task(self._redraw_forever())
        return self

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        if LiveView._active.get(type(self)) is self:
            del LiveView._active[type(self)]

    def _layout(self) -> LayoutDOM:
        raise NotImplementedError

    def _draw(self) -> None:
        raise NotImplementedError

    async def _redraw_forever(self) -> None:
        while True:
            self._draw_reporting_errors()
            await asyncio.sleep(1.0 / self.fps)

    def _draw_reporting_errors(self) -> None:
        try:
            self._draw()
        except Exception as error:
            message = f"[{type(self).__name__}] {type(error).__name__}: {error}"
            if message != self._last_error:
                print(message)
            self._last_error = message
        else:
            self._last_error = ""
