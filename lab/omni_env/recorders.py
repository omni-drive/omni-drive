"""Preconfigured telemetry recorders for laboratory tasks."""

import math
from collections import deque
from collections.abc import Callable

from omni_drive import PoseRecorder, SensorRecorder

from .context import (
    Q,
    R_us,
    d0_ir,
    d0_us,
    kalman,
    kf_optimal,
    kf_trust_odom,
    kf_trust_sensor,
    lidar_get_x,
    lidar_get_y,
    robot,
    x,
    z,
)

TASK_3 = "task_3"
TASK_3_ERROR = "task_3_error"
TASK_4 = "task_4"
TASK_5_2 = "task_5_2"
TASK_5_3 = "task_5_3"
TASK_6 = "task_6"
TASK_6_ERROR = "task_6_error"
TASK_6_TUNING = "task_6_tuning"

DEFAULT_LABELS = {
    "lidar": "Punkt odniesienia (LiDAR)",
    "infrared": "Czujnik podczerwieni",
    "ultrasonic": "Czujnik ultradźwiękowy",
    "kalman": "Filtr Kalmana",
    "kalman_us": "Filtr Kalmana (tylko US)",
    "kalman_fusion": "Filtr Kalmana (fuzja)",
    "odometry": "Odometria",
    "optimal": "Filtr optymalny",
    "trust_odom": "Zaufanie odometrii",
    "trust_sensor": "Zaufanie czujnikom",
}


def delayed[T](channel: Callable[[], T], samples: int = 1) -> Callable[[], T]:
    """Hold a FIFO buffer to delay channel readings by `samples` ticks, aligning phase with LiDAR latency."""
    buf: deque[T] = deque()

    def wrapper() -> T:
        buf.append(channel())
        if len(buf) > samples:
            return buf.popleft()
        return buf[0]

    return wrapper


def _kalman_us():
    """Background 1D Kalman filter (US-only baseline for Task 5.3)."""
    state_x, P, last_ox = 0.0, None, None

    def step():
        nonlocal state_x, P, last_ox
        ox = robot.pose.x
        q = Q.get()
        if P is None:
            P, last_ox = q, ox

        state_x += ox - last_ox
        P += q
        last_ox = ox

        meas_z = d0_us.get() - robot.us_front.distance
        if math.isfinite(meas_z):
            K = P / (P + R_us.get())
            state_x += K * (meas_z - state_x)
            P *= 1.0 - K
        return state_x

    return step


def _odom_x() -> float:
    return robot.pose.x


def _odom_xy() -> tuple[float, float]:
    return robot.pose.x, robot.pose.y


def _lidar_xy() -> tuple[float, float]:
    return lidar_get_x(), lidar_get_y()


def _error_mm(recorder: PoseRecorder, channel: str) -> float:
    pts, ref = recorder.data[channel], recorder.data["lidar"]
    if not pts or not ref:
        return float("nan")
    return math.hypot(pts[-1].x - ref[-1].x, pts[-1].y - ref[-1].y) * 1000.0


RECORDERS = {
    TASK_3: lambda: PoseRecorder(
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        odometry=delayed(_odom_xy),
        lidar=_lidar_xy,
        title="Odometria vs LiDAR",
    ),
    TASK_3_ERROR: lambda recorder: SensorRecorder(
        recorder.clock,
        title="Błąd względem LiDAR-u",
        y_label="Błąd [mm]",
        labels=DEFAULT_LABELS,
        odometry=lambda: _error_mm(recorder, "odometry"),
    ),
    TASK_4: lambda clock: SensorRecorder(
        clock,
        title="Ruch wahadłowy: LiDAR vs US vs IR",
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed", "ultrasonic": "line+scatter", "infrared": "line+scatter"},
        lidar=lambda: robot.lidar.beam_front,
        ultrasonic=delayed(lambda: robot.us_front.distance),
        infrared=delayed(lambda: robot.ir_front.distance),
    ),
    TASK_5_2: lambda clock: SensorRecorder(
        clock,
        title="Filtr Kalmana 1D (US) vs Odometria",
        y_label="Pozycja x [m]",
        labels=DEFAULT_LABELS,
        styles={"ultrasonic": "line+scatter", "kalman": "line", "odometry": "line", "lidar": "dashed"},
        odometry=delayed(_odom_x),
        ultrasonic=delayed(z.get),
        kalman=delayed(x.get),
        lidar=lidar_get_x,
    ),
    TASK_5_3: lambda clock: SensorRecorder(
        clock,
        title="Sekwencyjna fuzja (US + IR) vs Odometria",
        y_label="Pozycja x [m]",
        labels=DEFAULT_LABELS,
        styles={
            "infrared": "line+scatter",
            "kalman_us": "dashed",
            "kalman_fusion": "line",
            "odometry": "line",
            "lidar": "dashed",
        },
        odometry=delayed(_odom_x),
        infrared=delayed(lambda: d0_ir.get() - robot.ir_front.distance),
        kalman_us=delayed(_kalman_us()),
        kalman_fusion=delayed(x.get),
        lidar=lidar_get_x,
    ),
    TASK_6: lambda clock: PoseRecorder(
        clock,
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        odometry=delayed(_odom_xy),
        kalman=delayed(lambda: kalman.state),
        lidar=_lidar_xy,
        title="Odometria, Filtr Kalmana, LiDAR",
    ),
    TASK_6_ERROR: lambda recorder: SensorRecorder(
        recorder.clock,
        title="Błąd względem LiDAR-u",
        y_label="Błąd [mm]",
        labels=DEFAULT_LABELS,
        odometry=lambda: _error_mm(recorder, "odometry"),
        kalman=lambda: _error_mm(recorder, "kalman"),
    ),
    TASK_6_TUNING: lambda clock: PoseRecorder(
        clock,
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        optimal=delayed(lambda: kf_optimal.state),
        trust_odom=delayed(lambda: kf_trust_odom.state),
        trust_sensor=delayed(lambda: kf_trust_sensor.state),
        lidar=_lidar_xy,
        title="Wpływ strojenia Q i R w 2D",
    ),
}


def GET_RECORDER(name: str, *args, **kwargs):
    """Instantiate a preconfigured recorder by task name."""
    return RECORDERS[name](*args, **kwargs)
