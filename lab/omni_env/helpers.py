"""Helper functions for robot distance baseline calibration and setup."""

import time

import numpy as np

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


def _median(vals: list[float]) -> float:
    valid = [v for v in vals if v is not None and np.isfinite(v)]
    return float(np.median(valid)) if valid else float("nan")


def lidar_reset() -> tuple[float, float, float]:
    """Reset baseline lidar distance measurements from a single scan."""
    scan = robot.lidar
    return (
        f0.set(scan.beam_front),
        l0.set(scan.beam_left),
        r0.set(scan.beam_right),
    )


def median_distance(sensor, samples: int = 25, delay: float = 0.02) -> float:
    """Collect baseline distance measurements from a single sensor at rest and return their median."""
    readings = []
    for _ in range(samples):
        readings.append(sensor.distance)
        time.sleep(delay)
    return _median(readings)


def sensors_reset(samples: int = 25, delay: float = 0.02) -> tuple[float, float, float, float, float, float]:
    """Collect baseline distance measurements at rest and store their median."""
    ir_f, ir_l, ir_r = [], [], []
    us_f, us_l, us_r = [], [], []

    for _ in range(samples):
        ir_f.append(robot.ir_front.distance)
        ir_l.append(robot.ir_left.distance)
        ir_r.append(robot.ir_right.distance)
        us_f.append(robot.us_front.distance)
        us_l.append(robot.us_left.distance)
        us_r.append(robot.us_right.distance)
        time.sleep(delay)

    return (
        d0_ir_f.set(_median(ir_f)),
        d0_ir_l.set(_median(ir_l)),
        d0_ir_r.set(_median(ir_r)),
        d0_us_f.set(_median(us_f)),
        d0_us_l.set(_median(us_l)),
        d0_us_r.set(_median(us_r)),
    )


def rmse_2d(path, reference) -> float:
    """RMSE odległości między dwiema trajektoriami 2D (listy punktów (x, y))."""
    a, b = np.asarray(path, dtype=float), np.asarray(reference, dtype=float)
    n = min(len(a), len(b))
    sq_dist = np.sum((a[:n] - b[:n]) ** 2, axis=1)
    return float(np.sqrt(np.nanmean(sq_dist)))
