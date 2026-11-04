# omni_drive.clock

## Clock

```python
Clock(*, period: Time = Time.ms(100), duration: Time = Time.inf)
```

A periodic ticker for async control loops. Every `async for` loop over the same clock receives
the same ticks. Raises `ValueError` if `period` is not positive.

```python
clock = Clock(period=Time.ms(50), duration=Time.s(5))
async for t in clock:
    print(t.seconds)
```

| Member | Description |
| --- | --- |
| `async for t in clock` | yields a `Time`: the time elapsed since the clock started |
| `stop() -> None` | stops the clock and ends every loop over it |
| `period: Time`, `duration: Time` | as passed in |

Behaviour:

* The clock starts when the first loop begins iterating; the first tick is `Time.s(0)`. A loop
  that joins later receives the elapsed time since that start.
* Ticks are scheduled on a fixed grid, so the period does not drift. If a tick is late, the
  grid restarts from the late tick instead of sending a burst of catch-up ticks.
* Each loop holds at most one pending tick. A loop that is slower than `period` skips ticks and
  always gets the latest one.
* The clock stops after `duration`: the last tick has an elapsed time below `duration`, and
  then every loop ends.
* When the last loop exits, the clock stops. Iterating it again starts it again from 0.
* Interrupting the cell while the loop waits for the next tick ends the loop without raising, and
  code after the loop still runs. An interrupt during an `await` in the loop body (for example
  `await robot.drive_for(...)`) is raised as usual.

`DEFAULT_PERIOD = Time.ms(100)` and `DEFAULT_DURATION = Time.inf` are module constants.
