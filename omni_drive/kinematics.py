import math

HALF_SQRT3 = math.sqrt(3.0) / 2.0
SQRT3 = math.sqrt(3.0)


def wheel_speeds(vx: float, vy: float, omega: float) -> tuple[float, float, float]:
    return (
        -HALF_SQRT3 * vx + 0.5 * vy + omega,
        -vy + omega,
        HALF_SQRT3 * vx + 0.5 * vy + omega,
    )


def body_velocity(w1: float, w2: float, w3: float) -> tuple[float, float, float]:
    return (w3 - w1) / SQRT3, (w1 + w3 - 2.0 * w2) / 3.0, (w1 + w2 + w3) / 3.0
