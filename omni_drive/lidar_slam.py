import math
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation

from omni_drive.models import wrap_angle

LOG_ODDS_HIT = math.log(0.9 / 0.1)
LOG_ODDS_MISS = math.log(0.4 / 0.6)
LOG_ODDS_LIMIT = 6.0
MISS_CLEARANCE_CELLS = 2
OCCUPIED_PROBABILITY = 0.6
GAUSS_NEWTON_DAMPING = 1e-3
MAX_STEP_CELLS = 2.0
MAX_YAW_STEP_RAD = 0.2
CONVERGED_STEP_M = 5e-4
CONVERGED_STEP_RAD = 1e-3
MAX_POINT_STRIDE = 4
TIE_BREAK_WEIGHT = 1e-3


def to_world(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + (x, y)


@dataclass(frozen=True)
class MapSnapshot:
    version: int
    probability: np.ndarray
    x_min: float
    y_min: float
    resolution: float


class GridLevel:
    def __init__(self, resolution: float, cells: int, origin: float):
        self.resolution = resolution
        self.cells = cells
        self.origin = origin
        self.prob = np.full((cells, cells), 0.5, dtype=np.float32)

    def to_map(self, world: np.ndarray) -> np.ndarray:
        return (world - self.origin) / self.resolution

    def sample(self, map_points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        centred = map_points - 0.5
        corner = np.floor(centred)
        fx, fy = centred[:, 0] - corner[:, 0], centred[:, 1] - corner[:, 1]
        n = self.cells
        ix = np.clip(corner[:, 0].astype(np.intp), 0, n - 2)
        iy = np.clip(corner[:, 1].astype(np.intp), 0, n - 2)
        flat = ix * n + iy
        grid = self.prob.ravel()
        m00, m10 = grid[flat], grid[flat + n]
        m01, m11 = grid[flat + 1], grid[flat + n + 1]
        lower_row = m00 + fx * (m10 - m00)
        upper_row = m01 + fx * (m11 - m01)
        value = lower_row + fy * (upper_row - lower_row)
        gradient_x = (m10 - m00) + fy * ((m11 - m01) - (m10 - m00))
        gradient_y = upper_row - lower_row
        return value, gradient_x, gradient_y

    def lookup(self, map_points: np.ndarray) -> np.ndarray:
        index = np.floor(map_points).astype(np.int64)
        inside = np.all((index >= 0) & (index < self.cells), axis=-1)
        index = np.where(inside[..., None], index, 0)
        return np.where(inside, self.prob[index[..., 0], index[..., 1]], 0.5)


class OccupancyGrid:
    def __init__(self, resolution: float, size: float, levels: int = 1):
        block = 2 ** (levels - 1)
        cells = math.ceil(size / resolution / block) * block
        self.origin = -cells * resolution / 2
        self.levels = [
            GridLevel(resolution * 2**i, cells // 2**i, self.origin) for i in range(levels)
        ]
        self.bounds: tuple[np.ndarray, np.ndarray] | None = None
        self._log_odds = np.zeros((cells, cells), dtype=np.float32)
        self._block = block

    @property
    def resolution(self) -> float:
        return self.levels[0].resolution

    @property
    def cells(self) -> int:
        return self.levels[0].cells

    @property
    def prob(self) -> np.ndarray:
        return self.levels[0].prob

    def to_map(self, world: np.ndarray) -> np.ndarray:
        return self.levels[0].to_map(world)

    def lookup(self, map_points: np.ndarray) -> np.ndarray:
        return self.levels[0].lookup(map_points)

    def snapshot(self, version: int) -> MapSnapshot | None:
        if self.bounds is None:
            return None
        lo, hi = self.bounds
        return MapSnapshot(
            version=version,
            probability=self.prob[lo[0] : hi[0], lo[1] : hi[1]].copy(),
            x_min=self.origin + lo[0] * self.resolution,
            y_min=self.origin + lo[1] * self.resolution,
            resolution=self.resolution,
        )

    def raycast(
        self,
        origin: tuple[float, float],
        angles: np.ndarray,
        max_range: float,
        threshold: float = 0.65,
    ) -> np.ndarray:
        angles = np.atleast_1d(np.asarray(angles, dtype=float))
        step = self.resolution / 2
        distances = np.arange(step, max_range + step, step)
        directions = np.stack([np.cos(angles), np.sin(angles)], axis=-1)
        samples = np.asarray(origin, dtype=float) + distances[None, :, None] * directions[:, None]
        occupied = self.lookup(self.to_map(samples)) > threshold
        return np.where(occupied.any(axis=1), distances[occupied.argmax(axis=1)], np.inf)

    def integrate(self, sensor: tuple[float, float], endpoints: np.ndarray) -> None:
        start = self.to_map(np.asarray(sensor, dtype=float)[None, :])[0]
        ends = self.to_map(endpoints)
        hits = np.floor(ends).astype(np.int64)
        inside = np.all((hits >= 0) & (hits < self.cells), axis=1)
        ends, hits = ends[inside], hits[inside]
        if len(hits) == 0:
            return

        lo, hi = self._touched_box(np.vstack([hits, np.floor(start).astype(np.int64)]))
        box = self._log_odds[lo[0] : hi[0], lo[1] : hi[1]]
        hit_mask = np.zeros(box.shape, dtype=bool)
        hit_mask[hits[:, 0] - lo[0], hits[:, 1] - lo[1]] = True
        miss_mask = _traversed_cells(start - lo, ends[::2] - lo, box.shape)
        miss_mask &= ~binary_dilation(
            hit_mask, structure=np.ones((3, 3), dtype=bool), iterations=MISS_CLEARANCE_CELLS
        )
        box[miss_mask] += LOG_ODDS_MISS
        box[hit_mask] += LOG_ODDS_HIT
        np.clip(box, -LOG_ODDS_LIMIT, LOG_ODDS_LIMIT, out=box)
        self._refresh_pyramid(lo, hi)

    def _touched_box(self, cells: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        block = self._block
        lo = np.clip(cells.min(axis=0) // block * block, 0, self.cells)
        hi = np.clip((cells.max(axis=0) // block + 1) * block, 0, self.cells)
        if self.bounds is None:
            self.bounds = (lo.copy(), hi.copy())
        else:
            self.bounds = (np.minimum(self.bounds[0], lo), np.maximum(self.bounds[1], hi))
        return lo, hi

    def _refresh_pyramid(self, lo: np.ndarray, hi: np.ndarray) -> None:
        box = self._log_odds[lo[0] : hi[0], lo[1] : hi[1]]
        self.levels[0].prob[lo[0] : hi[0], lo[1] : hi[1]] = 1.0 / (1.0 + np.exp(-box))
        for i in range(1, len(self.levels)):
            f0, f1 = lo >> (i - 1), hi >> (i - 1)
            fine = self.levels[i - 1].prob[f0[0] : f1[0], f0[1] : f1[1]]
            h, w = fine.shape
            c0, c1 = lo >> i, hi >> i
            self.levels[i].prob[c0[0] : c1[0], c0[1] : c1[1]] = fine.reshape(
                h // 2, 2, w // 2, 2
            ).max(axis=(1, 3))


def _traversed_cells(start: np.ndarray, ends: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    rays = (ends - start).astype(np.float32)
    lengths = np.hypot(rays[:, 0], rays[:, 1])
    steps = np.maximum(lengths - 1.0, 0).astype(np.int64)
    total = int(steps.sum())
    if total == 0:
        return mask
    ray = np.repeat(np.arange(len(steps)), steps)
    first_step = np.cumsum(steps) - steps
    k = np.arange(total) - np.repeat(first_step, steps)
    fraction = (k.astype(np.float32) / np.maximum(lengths[ray], 1e-6))[:, None]
    cells = np.floor(start.astype(np.float32) + rays[ray] * fraction).astype(np.int64)
    cells = np.clip(cells, 0, np.array(shape) - 1)
    mask[cells[:, 0], cells[:, 1]] = True
    return mask


class LidarSlam:
    def __init__(
        self,
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
    ):
        self.resolution = resolution
        self.levels = levels
        self.map_size = map_size
        self.iterations = iterations
        self.min_score = min_score
        self.search_yaw = search_yaw
        self.search_yaw_step = search_yaw_step
        self.search_xy = search_xy
        self.search_stride = search_stride
        self.update_dist = update_dist
        self.update_yaw = update_yaw
        self.update_every_scans = update_every_scans
        self.reset()

    def reset(self) -> None:
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.map = OccupancyGrid(self.resolution, self.map_size, self.levels)
        self.score = 0.0
        self.confident = False
        self.map_version = 0
        self._last_integrated_pose: tuple[float, float, float] | None = None
        self._scans_since_integration = 0
        self._last_motion = np.zeros(3)
        self._last_heading_hint: float | None = None

    def snapshot(self) -> MapSnapshot | None:
        return self.map.snapshot(self.map_version)

    def update(
        self, points: np.ndarray, heading_hint: float | None = None
    ) -> tuple[float, float, float]:
        predicted = self._predict(heading_hint)
        if self._last_integrated_pose is None:
            self._integrate(points, predicted)
            return self._set_pose(predicted)

        pose = self._match(points, predicted)
        self.score = self._fraction_on_walls(points, pose)
        if self.score < self.min_score:
            recovered = self._match(points, self._search(points, predicted))
            recovered_score = self._fraction_on_walls(points, recovered)
            if recovered_score > self.score:
                pose, self.score = recovered, recovered_score
        self.confident = self.score >= self.min_score

        if not self.confident:
            self._last_motion[:] = 0.0
            return self._set_pose(predicted)

        self._last_motion = pose - (self.x, self.y, self.yaw)
        self._last_motion[2] = wrap_angle(self._last_motion[2])
        self._set_pose(pose)
        self._scans_since_integration += 1
        if self._should_integrate():
            self._integrate(points, np.array([self.x, self.y, self.yaw]))
        return self.x, self.y, self.yaw

    def _predict(self, heading_hint: float | None) -> np.ndarray:
        predicted = np.array([self.x, self.y, self.yaw]) + self._last_motion
        if heading_hint is not None:
            previous, self._last_heading_hint = self._last_heading_hint, heading_hint
            turned = 0.0 if previous is None else wrap_angle(heading_hint - previous)
            predicted[2] = self.yaw + turned
        return predicted

    def _set_pose(self, pose: np.ndarray) -> tuple[float, float, float]:
        self.x, self.y, self.yaw = float(pose[0]), float(pose[1]), wrap_angle(float(pose[2]))
        return self.x, self.y, self.yaw

    def _should_integrate(self) -> bool:
        x, y, yaw = self._last_integrated_pose
        moved = math.hypot(self.x - x, self.y - y) >= self.update_dist
        turned = abs(wrap_angle(self.yaw - yaw)) >= self.update_yaw
        return moved or turned or self._scans_since_integration >= self.update_every_scans

    def _integrate(self, points: np.ndarray, pose: np.ndarray) -> None:
        self.map.integrate((pose[0], pose[1]), to_world(points, *pose))
        self._last_integrated_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        self._scans_since_integration = 0
        self.map_version += 1

    def _fraction_on_walls(self, points: np.ndarray, pose: np.ndarray) -> float:
        noise_tolerant = self.map.levels[min(1, self.levels - 1)]
        value = noise_tolerant.lookup(noise_tolerant.to_map(to_world(points, *pose)))
        return float((value > OCCUPIED_PROBABILITY).mean())

    def _match(self, points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        pose = pose.astype(float).copy()
        for level in reversed(range(self.levels)):
            grid = self.map.levels[level]
            cells_per_m = 1.0 / grid.resolution
            max_step_m = MAX_STEP_CELLS * grid.resolution
            subset = points[:: min(2**level, MAX_POINT_STRIDE)]
            for _ in range(self.iterations[min(level, len(self.iterations) - 1)]):
                c, s = math.cos(pose[2]), math.sin(pose[2])
                value, gx, gy = grid.sample(grid.to_map(to_world(subset, *pose)))
                dx_dyaw = -s * subset[:, 0] - c * subset[:, 1]
                dy_dyaw = c * subset[:, 0] - s * subset[:, 1]
                jacobian = np.stack([gx, gy, gx * dx_dyaw + gy * dy_dyaw], axis=1) * cells_per_m
                hessian = jacobian.T @ jacobian + GAUSS_NEWTON_DAMPING * np.eye(3)
                try:
                    dx, dy, dyaw = np.linalg.solve(hessian, jacobian.T @ (1.0 - value)).tolist()
                except np.linalg.LinAlgError:
                    break
                pose[0] += np.clip(dx, -max_step_m, max_step_m)
                pose[1] += np.clip(dy, -max_step_m, max_step_m)
                pose[2] += np.clip(dyaw, -MAX_YAW_STEP_RAD, MAX_YAW_STEP_RAD)
                if max(abs(dx), abs(dy)) < CONVERGED_STEP_M and abs(dyaw) < CONVERGED_STEP_RAD:
                    break
        return pose

    def _search(self, points: np.ndarray, predicted: np.ndarray) -> np.ndarray:
        grid = self.map.levels[-1]
        subset = points[:: self.search_stride]
        yaws = predicted[2] + np.arange(
            -self.search_yaw, self.search_yaw + 1e-9, self.search_yaw_step
        )
        reach = round(self.search_xy / grid.resolution)
        offsets = np.arange(-reach, reach + 1)
        ox, oy = np.meshgrid(offsets, offsets, indexing="ij")
        shifts = np.stack([ox.ravel(), oy.ravel()], axis=1)

        c, s = np.cos(yaws), np.sin(yaws)
        rx = c[:, None] * subset[None, :, 0] - s[:, None] * subset[None, :, 1] + predicted[0]
        ry = s[:, None] * subset[None, :, 0] + c[:, None] * subset[None, :, 1] + predicted[1]
        rotated = grid.to_map(np.stack([rx, ry], axis=-1))
        candidates = rotated[:, None, :, :] + shifts[None, :, None, :]
        score = grid.lookup(candidates).sum(axis=-1)
        score -= TIE_BREAK_WEIGHT * (
            np.abs(yaws - predicted[2])[:, None] + np.hypot(*shifts.T)[None, :]
        )
        best_yaw, best_shift = np.unravel_index(np.argmax(score), score.shape)
        return np.array(
            [
                predicted[0] + shifts[best_shift, 0] * grid.resolution,
                predicted[1] + shifts[best_shift, 1] * grid.resolution,
                yaws[best_yaw],
            ]
        )
