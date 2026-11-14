"""Preconfigured telemetry recorders for laboratory tasks."""

import sys
from collections import deque
from collections.abc import Callable

from omni_drive import PoseRecorder, SensorRecorder


def delayed[T](channel: Callable[[], T], samples: int = 2) -> Callable[[], T]:
    """Hold a FIFO buffer to delay channel readings by `samples` ticks, aligning phase with LiDAR latency."""
    buf: deque[T] = deque()

    def wrapper() -> T:
        val = channel()
        buf.append(val)
        if len(buf) > samples:
            return buf.popleft()
        return buf[0]

    return wrapper


class _Var:
    """Dynamic proxy resolving attributes and calls lazily from the notebook's __main__ scope."""

    def __init__(self, name: str, default=0.0):
        self._name = name
        self._default = default

    def _val(self):
        return getattr(sys.modules["__main__"], self._name, self._default)

    def __getattr__(self, item: str):
        return getattr(self._val(), item)

    def __call__(self, *args, **kwargs):
        return self._val()(*args, **kwargs)

    def __float__(self) -> float:
        return float(self._val())


# Notebook scope proxies
robot = _Var("robot")
kalman = _Var("kalman")
kf_optimal = _Var("kf_optimal")
kf_trust_odom = _Var("kf_trust_odom")
kf_trust_sensor = _Var("kf_trust_sensor")
lidar_get_x = _Var("lidar_get_x")
lidar_get_y = _Var("lidar_get_y")
x = _Var("x")
z = _Var("z")
x_ir = _Var("x_ir")
z_us = _Var("z_us")
x_fusion = _Var("x_fusion")
d0_ir_f = _Var("d0_ir_f")
d0_ir_l = _Var("d0_ir_l")
d0_ir_r = _Var("d0_ir_r")


def ir_get_x() -> float:
    return float(d0_ir_f) - robot.ir_front.distance


def ir_get_y() -> float:
    dl = float(d0_ir_l) - robot.ir_left.distance
    dr = float(d0_ir_r) - robot.ir_right.distance
    return 0.5 * (dl - dr)


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
    "kalman_ir": "Filtr Kalmana (tylko IR)",
    "kalman_fusion": "Filtr Kalmana (fuzja)",
    "odometry": "Odometria",
    "optimal": "Filtr optymalny",
    "trust_odom": "Zaufanie odometrii",
    "trust_sensor": "Zaufanie czujnikom",
}


RECORDERS = {
    TASK_3: lambda: PoseRecorder(
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        odometry=delayed(lambda: (robot.pose.x, robot.pose.y)),
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
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
        title="Filtr Kalmana 1D (IR) vs Odometria",
        y_label="Pozycja x [m]",
        labels=DEFAULT_LABELS,
        styles={"infrared": "line+scatter", "kalman": "line", "odometry": "line", "lidar": "dashed"},
        odometry=delayed(lambda: robot.pose.x),
        infrared=delayed(lambda: z),
        kalman=delayed(lambda: x),
        lidar=lambda: lidar_get_x(),
    ),
    TASK_5_3: lambda clock: SensorRecorder(
        clock,
        title="Sekwencyjna fuzja (IR + US) vs Odometria",
        y_label="Pozycja x [m]",
        labels=DEFAULT_LABELS,
        styles={
            "ultrasonic": "line+scatter",
            "kalman_ir": "dashed",
            "kalman_fusion": "line",
            "odometry": "line",
            "lidar": "dashed",
        },
        odometry=delayed(lambda: robot.pose.x),
        ultrasonic=delayed(lambda: z_us),
        kalman_ir=delayed(lambda: x_ir),
        kalman_fusion=delayed(lambda: x_fusion),
        lidar=lambda: lidar_get_x(),
    ),
    TASK_6: lambda clock: PoseRecorder(
        clock,
        labels=DEFAULT_LABELS,
        styles={"infrared": "dashed+scatter", "lidar": "dashed"},
        odometry=delayed(lambda: (robot.pose.x, robot.pose.y)),
        infrared=delayed(lambda: (ir_get_x(), ir_get_y())),
        kalman=delayed(lambda: kalman.state),
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
        title="Odometria, Filtr Kalmana, LiDAR",
    ),
    TASK_6_TUNING: lambda: PoseRecorder(
        labels=DEFAULT_LABELS,
        styles={"lidar": "dashed"},
        optimal=delayed(lambda: kf_optimal.state),
        trust_odom=delayed(lambda: kf_trust_odom.state),
        trust_sensor=delayed(lambda: kf_trust_sensor.state),
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
        title="Wpływ strojenia Q i R w 2D",
    ),
}


def GET_RECORDER(name: str, *args, **kwargs):
    """Instantiate a preconfigured recorder by task name."""
    return RECORDERS[name](*args, **kwargs)
