"""Preconfigured telemetry recorders for laboratory tasks."""

import math
from collections import deque
from collections.abc import Callable

from omni_drive import PoseRecorder, SensorRecorder

from .context import (
    Q,
    R_us,
    _Var,
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
    z_ir,
)


def delayed[T](channel: Callable[[], T], samples: int = 1) -> Callable[[], T]:
    """Hold a FIFO buffer to delay channel readings by `samples` ticks, aligning phase with LiDAR latency."""
    buf: deque[T] = deque()

    def wrapper() -> T:
        val = channel()
        if isinstance(val, _Var):
            val = val.get()
        buf.append(val)
        if len(buf) > samples:
            return buf.popleft()
        return buf[0]

    return wrapper


TASK_3 = "task_3"
TASK_4 = "task_4"
TASK_5_2 = "task_5_2"
TASK_5_3 = "task_5_3"
TASK_6 = "task_6"
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


def _kalman_us():
    """Background 1D Kalman filter (US-only baseline for Task 5.3)."""
    x, P, last_ox = 0.0, None, None

    def step():
        nonlocal x, P, last_ox
        ox = robot.pose.x
        if P is None:
            P, last_ox = Q.get(), ox

        x += ox - last_ox
        P += Q
        last_ox = ox

        z = d0_us - robot.us_front.distance
        if math.isfinite(z):
            K = P / (P + R_us)
            x += K * (z - x)
            P *= 1.0 - K
        return x

    return step



def _odom_x() -> float:
    return robot.pose.x


def _odom_xy() -> tuple[float, float]:
    return robot.pose.x, robot.pose.y


def _lidar_xy() -> tuple[float, float]:
    return lidar_get_x(), lidar_get_y()


RECORDERS = {
    TASK_3: lambda clock=None: PoseRecorder(
        clock,
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        odometry=delayed(_odom_xy),
        lidar=_lidar_xy,
        title="Odometria vs LiDAR",
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
        ultrasonic=delayed(lambda: z),
        kalman=delayed(lambda: x),
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
        infrared=delayed(lambda: z_ir),
        kalman_us=delayed(_kalman_us()),
        kalman_fusion=delayed(lambda: x),
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
    TASK_6_TUNING: lambda clock=None: PoseRecorder(
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


def GET_ERROR_RECORDER(pose_rec, clock=None, reference: str = "lidar"):
    """Live companion of a 2D recorder, shown next to it with LivePlot(rec, err):
    distance of each recorded trajectory from the reference, in millimetres."""

    def error_of(name):
        def channel():
            data = pose_rec.data
            if not data.get(name) or not data.get(reference):
                return math.nan
            a, b = data[name][-1], data[reference][-1]
            return math.hypot(a[0] - b[0], a[1] - b[1]) * 1000

        return channel

    names = [n for n in pose_rec.channels if n != reference]
    return SensorRecorder(
        clock if clock is not None else pose_rec.clock,
        title="Błąd położenia względem LiDAR-u",
        y_label="błąd [mm]",
        labels=DEFAULT_LABELS,
        **{n: error_of(n) for n in names},
    )
