"""Notebook context dynamic variable proxies."""

import sys


class _Var:
    """Dynamic proxy resolving attributes and calls lazily from the notebook's __main__ scope."""

    def __init__(self, name: str):
        self._name = name

    def get(self):
        return getattr(sys.modules["__main__"], self._name)

    def set[T](self, val: T) -> T:
        setattr(sys.modules["__main__"], self._name, val)
        return val

    def __getattr__(self, item: str):
        return getattr(self.get(), item)

    def __call__(self, *args, **kwargs):
        return self.get()(*args, **kwargs)


# Notebook scope proxies (in order of appearance in notebook.ipynb)

# Setup
robot = _Var("robot")

# Task 3 – LiDAR & process noise
f0 = _Var("f0")
l0 = _Var("l0")
r0 = _Var("r0")
lidar_get_x = _Var("lidar_get_x")
lidar_get_y = _Var("lidar_get_y")
Q = _Var("Q")

# Task 4 – Measurement noise
R_ir = _Var("R_ir")
R_us = _Var("R_us")

# Task 5.2 – Kalman 1D (US)
x = _Var("x")
z = _Var("z")
d0_us = _Var("d0_us")

# Task 5.3 – Kalman 1D fusion (US + IR)
d0_ir = _Var("d0_ir")

# Task 6 – Kalman 2D
d0_ir_f = _Var("d0_ir_f")
d0_ir_l = _Var("d0_ir_l")
d0_ir_r = _Var("d0_ir_r")
d0_us_f = _Var("d0_us_f")
d0_us_l = _Var("d0_us_l")
d0_us_r = _Var("d0_us_r")
kalman = _Var("kalman")

# Task 6 – Tuning
kf_optimal = _Var("kf_optimal")
kf_trust_odom = _Var("kf_trust_odom")
kf_trust_sensor = _Var("kf_trust_sensor")
