"""Sensor baselines and trajectory error helpers for the lab notebook."""

import time

import numpy as np
from omni_drive import Distance

from .context import (
    d0_ir_f,
    d0_ir_l,
    d0_ir_r,
    d0_us_f,
    d0_us_l,
    d0_us_r,
    f0,
    l0,
    r0,
    robot,
)


def _medians(reads, samples: int, delay: float) -> list[Distance]:
    """Median of `samples` readings of each function in `reads`, ignoring missing readings."""
    rows = []
    for _ in range(samples):
        rows.append([float(read()) for read in reads])
        time.sleep(delay)
    result = []
    for column in np.array(rows, dtype=float).T:
        finite = column[np.isfinite(column)]
        result.append(Distance.m(float(np.median(finite))) if finite.size else Distance.infinite())
    return result


def lidar_reset(samples: int = 25, delay: float = 0.04) -> tuple[Distance, Distance, Distance]:
    """Store the median front, left and right LiDAR distances at rest as f0, l0, r0."""
    front, left, right = _medians(
        [lambda: robot.lidar.beam_front, lambda: robot.lidar.beam_left, lambda: robot.lidar.beam_right],
        samples,
        delay,
    )
    return f0.set(front), l0.set(left), r0.set(right)


def median_distance(sensor, samples: int = 25, delay: float = 0.04) -> Distance:
    """Median distance of one sensor at rest."""
    return _medians([lambda: sensor.distance], samples, delay)[0]


def sensors_reset(samples: int = 25, delay: float = 0.04) -> tuple[Distance, ...]:
    """Store the median distances of all six sensors at rest as d0_ir_f ... d0_us_r."""
    sensors = [robot.ir_front, robot.ir_left, robot.ir_right, robot.us_front, robot.us_left, robot.us_right]
    values = _medians([lambda s=s: s.distance for s in sensors], samples, delay)
    names = (d0_ir_f, d0_ir_l, d0_ir_r, d0_us_f, d0_us_l, d0_us_r)
    return tuple(name.set(value) for name, value in zip(names, values, strict=True))


def rmse_2d(path, reference) -> float:
    """RMSE odległości między dwiema trajektoriami 2D (listy punktów (x, y))."""
    a, b = np.asarray(path, dtype=float), np.asarray(reference, dtype=float)
    n = min(len(a), len(b))
    sq_dist = np.sum((a[:n] - b[:n]) ** 2, axis=1)
    return float(np.sqrt(np.nanmean(sq_dist)))
