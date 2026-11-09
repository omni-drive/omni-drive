"""Preconfigured telemetry recorders for laboratory tasks."""

import sys

from omni_drive import PoseRecorder, SensorRecorder


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


RECORDERS = {
    "task_3": lambda: PoseRecorder(
        odometry=lambda: (robot.pose.x, robot.pose.y),
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
        title="Odometria vs LiDAR",
    ),
    "task_4": lambda clock: SensorRecorder(
        clock,
        title="Ruch wahadłowy: LiDAR vs US vs IR",
        styles={"lidar": "dashed", "ultrasonic": "line+scatter", "infrared": "line+scatter"},
        lidar=lambda: robot.lidar.beam_front,
        ultrasonic=lambda: robot.us_front.distance,
        infrared=lambda: robot.ir_front.distance,
    ),
    "task_5_2": lambda clock: SensorRecorder(
        clock,
        title="Filtr Kalmana 1D (IR) vs Odometria",
        y_label="Pozycja x [m]",
        styles={"infrared": "line+scatter", "kalman": "line", "odometry": "line", "lidar": "dashed"},
        odometry=lambda: robot.pose.x,
        infrared=lambda: z,
        kalman=lambda: x,
        lidar=lambda: lidar_get_x(),
    ),
    "task_5_3": lambda clock: SensorRecorder(
        clock,
        title="Sekwencyjna fuzja (IR + US) vs Odometria",
        y_label="Pozycja x [m]",
        styles={
            "ultrasonic": "line+scatter",
            "kalman_ir": "dashed",
            "kalman_fusion": "line",
            "odometry": "line",
            "lidar": "dashed",
        },
        odometry=lambda: robot.pose.x,
        ultrasonic=lambda: z_us,
        kalman_ir=lambda: x_ir,
        kalman_fusion=lambda: x_fusion,
        lidar=lambda: lidar_get_x(),
    ),
    "task_6": lambda clock: PoseRecorder(
        clock,
        odometry=lambda: (robot.pose.x, robot.pose.y),
        kalman=lambda: kalman.state,
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
        title="Odometria, Filtr Kalmana, LiDAR",
    ),
    "task_6_tuning": lambda: PoseRecorder(
        optimal=lambda: kf_optimal.state,
        trust_odom=lambda: kf_trust_odom.state,
        trust_sensor=lambda: kf_trust_sensor.state,
        lidar=lambda: (lidar_get_x(), lidar_get_y()),
        title="Wpływ strojenia Q i R w 2D",
    ),
}


def GET_RECORDER(name: str, *args, **kwargs):
    """Instantiate a preconfigured recorder by task name."""
    return RECORDERS[name](*args, **kwargs)
