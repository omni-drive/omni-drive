import math
from dataclasses import dataclass
from enum import Enum


class WheelId(Enum):
    WHEEL_1 = 0
    WHEEL_2 = 1
    WHEEL_3 = 2


class UltrasonicSensorId(Enum):
    US_1 = 0
    US_2 = 1
    US_3 = 2


@dataclass(frozen=True)
class Time:
    seconds: float

    @classmethod
    def s(cls, value: float) -> "Time":
        return cls(value)

    @classmethod
    def ms(cls, value: float) -> "Time":
        return cls(value / 1000)


@dataclass(frozen=True)
class Distance:
    meters: float

    @classmethod
    def m(cls, value: float) -> "Distance":
        return cls(value)

    @classmethod
    def cm(cls, value: float) -> "Distance":
        return cls(value / 100)

    @classmethod
    def mm(cls, value: float) -> "Distance":
        return cls(value / 1000)

    @property
    def as_cm(self) -> float:
        return self.meters * 100

    @property
    def as_mm(self) -> float:
        return self.meters * 1000


@dataclass(frozen=True)
class Angle:
    radians: float

    @classmethod
    def rad(cls, value: float) -> "Angle":
        return cls(value)

    @classmethod
    def deg(cls, value: float) -> "Angle":
        return cls(math.radians(value))

    @classmethod
    def rev(cls, value: float) -> "Angle":
        return cls(value * math.tau)

    @property
    def degrees(self) -> float:
        return math.degrees(self.radians)

    @property
    def revolutions(self) -> float:
        return self.radians / math.tau


@dataclass(frozen=True)
class DriveVector:
    velocity: float
    heading: Angle
    omega: float = 0.0

    @property
    def vx(self) -> float:
        return self.velocity * math.cos(self.heading.radians)

    @property
    def vy(self) -> float:
        return self.velocity * math.sin(self.heading.radians)

    @classmethod
    def zero(cls) -> "DriveVector":
        return cls(velocity=0.0, heading=Angle(0.0))


def wrap_angle(radians: float) -> float:
    return (radians + math.pi) % math.tau - math.pi


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def compose(self, local: "Pose") -> "Pose":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose(
            self.x + c * local.x - s * local.y,
            self.y + s * local.x + c * local.y,
            wrap_angle(self.yaw + local.yaw),
        )

    def relative_to(self, origin: "Pose") -> "Pose":
        c, s = math.cos(origin.yaw), math.sin(origin.yaw)
        dx, dy = self.x - origin.x, self.y - origin.y
        return Pose(c * dx + s * dy, -s * dx + c * dy, wrap_angle(self.yaw - origin.yaw))

    def distance_to(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def advanced(self, dx: float, dy: float, dyaw: float) -> "Pose":
        mid_yaw = self.yaw + dyaw / 2
        c, s = math.cos(mid_yaw), math.sin(mid_yaw)
        return Pose(self.x + c * dx - s * dy, self.y + s * dx + c * dy, wrap_angle(self.yaw + dyaw))

    def increment_to(self, target: "Pose") -> tuple[float, float, float]:
        step = target.relative_to(self)
        c, s = math.cos(step.yaw / 2), math.sin(step.yaw / 2)
        return c * step.x + s * step.y, -s * step.x + c * step.y, step.yaw
