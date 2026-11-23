# omni_drive.units

Physical quantities with units. Import them from `omni_drive`:

```python
from omni_drive import Angle, Distance, Speed, Time
```

Each class is a frozen dataclass holding one value in SI units. The arithmetic rules are the
same for all of them and are explained in [Frames and units](../concepts/frames.md#units).

## Distance

```python
Distance(meters: float)
```

| Member | Description |
| --- | --- |
| `Distance.m(value)`, `Distance.cm(value)`, `Distance.mm(value)` | from meters, centimeters, millimeters |
| `Distance.zero()`, `Distance.infinite()` | 0 m; an infinite distance (sensors return it when there is no reading) |
| `meters: float` | the value in meters |
| `centimeters: float`, `millimeters: float` | the value in centimeters, millimeters (also `as_cm`, `as_mm`) |

## Angle

```python
Angle(radians: float)
```

Robot convention: 0° is forward, positive angles turn clockwise (90° right, −90° left).

| Member | Description |
| --- | --- |
| `Angle.deg(value)`, `Angle.rad(value)`, `Angle.rev(value)` | from degrees, radians, revolutions (1 = 360°) |
| `Angle.zero()` | 0° |
| `radians: float` | the value in radians, as given (not wrapped) |
| `degrees: float`, `revolutions: float` | the value in degrees, revolutions (not wrapped) |
| `wrapped() -> Angle` | the same direction in [−180°, 180°) |
| `sin() -> float`, `cos() -> float` | sine and cosine; `math.sin(angle)` works too |

An `Angle` prints in degrees: `f"{Angle.rad(math.pi):.1f}"` gives `180.0°`.

## Time

```python
Time(seconds: float)
```

| Member | Description |
| --- | --- |
| `Time.s(value)`, `Time.ms(value)` | from seconds, milliseconds |
| `Time.zero()`, `Time.infinite()` | 0 s; an infinite duration |
| `seconds: float`, `milliseconds: float` | the value in seconds, milliseconds |

## Speed

```python
Speed(meters_per_second: float)
```

| Member | Description |
| --- | --- |
| `Speed.mps(value)`, `Speed.cmps(value)` | from meters per second, centimeters per second |
| `meters_per_second: float` | the value in m/s |

## Common members

| Member | Description |
| --- | --- |
| `a + b`, `a - b`, `<`, `<=`, `>`, `>=` | only between values of the same type; otherwise `TypeError` |
| `a * n`, `n * a`, `a / n` | scale by a plain number, same type |
| `a / b` | same type: a plain number; `Distance / Time` → `Speed`; `Distance / Speed` → `Time` |
| `Speed * Time`, `Time * Speed` | a `Distance` |
| `Distance * Distance`, `a ** n` | a plain number (`Distance ** 2` is in m²) |
| `-a`, `abs(a)`, `round(a, n)`, `sum([...])` | same type |
| `a == b` | `True` only for the same type and value; `Distance.cm(5) == 0.05` is `False` |
| `float(a)` | the SI value |
| `si: float` | the SI value |
| `is_finite: bool` | `False` for infinite values |

`seconds(value)` and `require(value, kind, name)` are helpers used by the library itself.
