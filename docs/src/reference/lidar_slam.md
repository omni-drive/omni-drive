# omni_drive.lidar_slam

2D LiDAR SLAM. The module builds an occupancy-grid map and tracks the robot's pose by matching
each scan against that map. `Localizer` runs it automatically; direct use is intended only for
experiments with recorded scans.

The map is a log-odds occupancy grid. Each LiDAR ray increases the free probability of the cells
it crosses and the occupied probability of its end cell. Cells within 2 cells of a hit in the
same scan are not marked free, so rays that graze a wall do not erase it. For matching, the
module also keeps coarser copies of the grid (2 × 2 max-pooled).

Each scan is aligned with the map, not with the previous scan, so errors do not accumulate from
scan to scan. The matcher runs Gauss-Newton over (x, y, yaw) on the smoothly interpolated grid,
coarse level first. This is the method of Hector SLAM (Kohlbrecher et al. 2011).

If the result scores below `min_score`, a brute-force search over a window of poses on the
coarsest grid selects a new starting pose and Gauss-Newton runs again, as in Cartographer's
correlative scan matcher. If this also fails, the matcher reports low confidence and returns its
prediction.

A scan is added to the map only when the robot has moved `update_dist` or turned `update_yaw`
since the last update, or after `update_every_scans` scans.

The module does not implement loop closure. The map limits drift but does not correct it
retrospectively.

## LidarSlam

```python
LidarSlam(
    resolution: float = 0.03,
    levels: int = 3,
    map_size: float = 24.0,
    iterations: tuple[int, ...] = (4, 3, 3),
    min_score: float = 0.4,
    search_yaw: float = math.radians(45),
    search_yaw_step: float = math.radians(2),
    search_xy: float = 0.25,
    search_stride: int = 3,
    update_dist: float = 0.05,
    update_yaw: float = math.radians(3),
    update_every_scans: int = 20,
)
```

| Parameter | Description |
| --- | --- |
| `resolution` | cell size of the finest grid, meters |
| `levels` | number of grid resolutions, each twice as coarse as the one before |
| `map_size` | side of the square map, meters, centered on the start |
| `iterations` | Gauss-Newton iterations per level, finest first |
| `min_score` | score below which the matcher runs the recovery search and reports low confidence |
| `search_yaw`, `search_yaw_step` | half-width and step of the recovery search in heading, radians |
| `search_xy` | half-width of the recovery search in position, meters |
| `search_stride` | the search uses every n-th scan point |
| `update_dist`, `update_yaw`, `update_every_scans` | conditions for adding a scan to the map, described above |

| Member | Description |
| --- | --- |
| `update(points: np.ndarray, heading_hint: float | None = None) -> tuple[float, float, float]` | matches one scan, an `(N, 2)` array in meters in the robot frame, and returns the new `(x, y, yaw)`. `heading_hint`, if given, is an absolute heading from the gyroscope or odometry; only its change since the previous call is used, as the predicted turn |
| `reset() -> None` | clears the map and puts the robot back at `(0, 0, 0)` |
| `x`, `y`, `yaw` | the current pose |
| `score: float` | fraction of the last scan's points on occupied cells |
| `confident: bool` | `score >= min_score` |
| `map: OccupancyGrid` | the map |
| `map_version: int` | increases each time a scan is added to the map |
| `snapshot() -> MapSnapshot | None` | a copy of the mapped area; `None` before the first scan |

## OccupancyGrid

```python
OccupancyGrid(resolution: float, size: float, levels: int = 1)
```

A square grid centered on the origin of the start frame.

| Member | Description |
| --- | --- |
| `prob: np.ndarray` | occupancy probability per cell of the finest level, indexed `[ix, iy]`; 0.5 is unknown |
| `resolution: float` | cell size, meters |
| `cells: int` | cells per side |
| `origin: float` | world coordinate of the grid's lower edge, the same for x and y |
| `bounds` | `(lo, hi)` cell indices of the area any scan touched, or `None` |
| `to_map(world: np.ndarray) -> np.ndarray` | world meters to continuous cell coordinates |
| `lookup(map_pts: np.ndarray) -> np.ndarray` | occupancy of the cell under each point; 0.5 outside the grid |
| `raycast(origin, angles, max_range, threshold=0.65) -> np.ndarray` | for each world angle, the distance from `origin` to the first cell with occupancy above `threshold`; `inf` if none within `max_range` |
| `integrate(sensor, endpoints) -> None` | adds one scan: `sensor` is the LiDAR position and `endpoints` the `(N, 2)` hit points, both in world meters |
| `snapshot(version: int) -> MapSnapshot | None` | a copy of the area inside `bounds`, labeled with `version` |

```python
grid = loc.slam.map  # live; use loc.map_snapshot() for a stable copy
cell = grid.to_map(np.array([[1.0, 0.5]]))  # where (1.0 m, 0.5 m) falls in the grid
grid.lookup(cell)                           # its occupancy probability
```

## MapSnapshot

```python
MapSnapshot(version: int, probability: np.ndarray, x_min: float, y_min: float, resolution: float)
```

A copy of the mapped part of the grid that remains valid while the localizer continues to update
the map. The dataclass is frozen.

| Field | Description |
| --- | --- |
| `version` | the map version it was taken at |
| `probability` | occupancy probability per cell, indexed `[ix, iy]`: the first index follows world x, the second world y |
| `x_min`, `y_min` | world coordinates of the lower corner of cell `[0, 0]`, meters |
| `resolution` | cell size, meters |

```python
snap = loc.map_snapshot()
if snap is not None:
    occupied = snap.probability > 0.65
    ix, iy = np.nonzero(occupied)
    wall_x = snap.x_min + (ix + 0.5) * snap.resolution  # cell centers, meters
    wall_y = snap.y_min + (iy + 0.5) * snap.resolution
```

## to_world

```python
to_world(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray
```

Transforms `(N, 2)` points from the robot frame into the world frame of a robot at
`(x, y, yaw)`.
