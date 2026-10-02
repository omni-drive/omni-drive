function normalizeSelection(indices, pointCount) {
    return Array.from(indices || []).filter(
        index => Number.isInteger(index) && index >= 0 && index < pointCount
    );
}

function findChangedPoint(pointsX, pointsY, previousX, previousY) {
    if (pointsX.length !== previousX.length) {
        return pointsX.length > previousX.length ? pointsX.length - 1 : -1;
    }

    for (let index = 0; index < pointsX.length; index++) {
        if (
            Math.abs(pointsX[index] - previousX[index]) > 1e-4 ||
            Math.abs(pointsY[index] - previousY[index]) > 1e-4
        ) {
            return index;
        }
    }

    return -1;
}

function resolveActiveIndex(
    selected,
    newlySelected,
    changedIndex,
    previousActiveIndex,
    pointCount
) {
    if (changedIndex >= 0) return changedIndex;
    if (newlySelected.length > 0) return newlySelected[newlySelected.length - 1];
    if (previousActiveIndex >= 0 && previousActiveIndex < pointCount) {
        return previousActiveIndex;
    }
    if (selected.length > 0) return selected[0];
    return pointCount > 0 ? pointCount - 1 : -1;
}

function getSliderSpeed(selected, activeIndex, pointSpeeds) {
    if (selected.length === 1) return pointSpeeds[selected[0]];
    if (selected.length > 1) {
        const firstSpeed = pointSpeeds[selected[0]];
        return selected.every(index => pointSpeeds[index] === firstSpeed)
            ? firstSpeed
            : pointSpeeds[activeIndex];
    }
    return activeIndex >= 0 ? pointSpeeds[activeIndex] : null;
}

function updateSliderSpeed(selected, activeIndex, pointSpeeds) {
    const sliderSpeed = getSliderSpeed(selected, activeIndex, pointSpeeds);
    if (sliderSpeed === null || sliderSpeed === undefined) return;

    if (speed_slider.value !== sliderSpeed) {
        speed_slider._pending_sync = sliderSpeed;
        speed_slider.value = sliderSpeed;
    } else {
        delete speed_slider._pending_sync;
    }
}

function updateSliderTitle(selected, activeIndex, pointSpeeds) {
    if (selected.length > 1) {
        const firstSpeed = pointSpeeds[selected[0]];
        const allSame = selected.every(index => pointSpeeds[index] === firstSpeed);
        speed_slider.title = allSame
            ? `Speed (${selected.length} selected) [${Math.round(firstSpeed)}%]`
            : `Speed (${selected.length} selected) [Multiple values]`;
    } else if (activeIndex >= 0 && activeIndex < pointSpeeds.length) {
        speed_slider.title = `Point speed #${activeIndex + 1} [${Math.round(pointSpeeds[activeIndex])}%]`;
    } else {
        speed_slider.title = "Point speed";
    }
}

function hexToRgb(hex) {
    const value = parseInt(hex.slice(1), 16);
    return [(value >> 16) & 255, (value >> 8) & 255, value & 255];
}

function blendColor(from, to, ratio) {
    const [r, g, b] = from.map((channel, index) => Math.round(channel + (to[index] - channel) * ratio));
    return `rgb(${r},${g},${b})`;
}

function speedToColor(value) {
    const [slow, medium, fast] = speed_colors.map(hexToRgb);
    const clamped = Math.max(min_speed, Math.min(max_speed, value));
    if (clamped <= default_speed) {
        return blendColor(slow, medium, (clamped - min_speed) / Math.max(1, default_speed - min_speed));
    }
    return blendColor(medium, fast, (clamped - default_speed) / Math.max(1, max_speed - default_speed));
}

function catmullRomPoint(p0, p1, p2, p3, t) {
    const t2 = t * t;
    const t3 = t2 * t;
    const c0 = -0.5 * t3 + t2 - 0.5 * t;
    const c1 = 1.5 * t3 - 2.5 * t2 + 1.0;
    const c2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
    const c3 = 0.5 * t3 - 0.5 * t2;

    return {
        x: c0 * p0.x + c1 * p1.x + c2 * p2.x + c3 * p3.x,
        y: c0 * p0.y + c1 * p1.y + c2 * p2.y + c3 * p3.y,
    };
}

function blendPoint(linearPoint, splinePoint, smooth) {
    return {
        x: linearPoint.x * (1.0 - smooth) + splinePoint.x * smooth,
        y: linearPoint.y * (1.0 - smooth) + splinePoint.y * smooth,
    };
}

function buildTrajectorySegment(points, startSpeed, endSpeed, smooth) {
    const xs = [];
    const ys = [];
    const colors = [];
    let totalDistance = 0.0;
    let previousPoint = points[1];

    for (let sample = 1; sample <= samples; sample++) {
        const t = sample / samples;
        const splinePoint = catmullRomPoint(points[0], points[1], points[2], points[3], t);
        const linearPoint = {
            x: points[1].x + t * (points[2].x - points[1].x),
            y: points[1].y + t * (points[2].y - points[1].y),
        };
        const currentPoint = blendPoint(linearPoint, splinePoint, smooth);
        const currentSpeed = startSpeed + t * (endSpeed - startSpeed);
        totalDistance += Math.hypot(currentPoint.x - previousPoint.x, currentPoint.y - previousPoint.y);
        xs.push([previousPoint.x, currentPoint.x]);
        ys.push([previousPoint.y, currentPoint.y]);
        colors.push(speedToColor(currentSpeed));
        previousPoint = currentPoint;
    }

    return { xs, ys, colors, totalDistance };
}

function buildTrajectory(robotX, robotY, pointsX, pointsY, pointSpeeds) {
    const route = pointsX.map((x, index) => ({ x, y: pointsY[index] }));
    route.unshift({ x: robotX, y: robotY });
    const routeSpeed = [pointSpeeds[0] ?? default_speed, ...pointSpeeds];
    const smooth = Math.max(0.0, Math.min(100.0, smoothing_slider.value)) / 100.0;
    const curve = { xs: [], ys: [], colors: [], totalDistance: 0.0 };

    for (let index = 0; index < route.length - 1; index++) {
        const segment = buildTrajectorySegment(
            [
                index > 0 ? route[index - 1] : route[index],
                route[index],
                route[index + 1],
                index + 2 < route.length ? route[index + 2] : route[index + 1],
            ],
            routeSpeed[index],
            routeSpeed[index + 1],
            smooth
        );
        curve.xs.push(...segment.xs);
        curve.ys.push(...segment.ys);
        curve.colors.push(...segment.colors);
        curve.totalDistance += segment.totalDistance;
    }

    return curve;
}

const pointsX = Array.from(waypoints.data.x || []);
const pointsY = Array.from(waypoints.data.y || []);
let pointSpeeds = Array.from(speeds.data.speed || []);
const state = waypoints._ui_state || {
    previousX: [],
    previousY: [],
    previousSelected: [],
    activeIndex: -1,
};

while (pointSpeeds.length < pointsX.length) pointSpeeds.push(default_speed);
if (pointSpeeds.length > pointsX.length) pointSpeeds.length = pointsX.length;

const robotX = robot.data.x?.[0] ?? 0.0;
const robotY = robot.data.y?.[0] ?? 0.0;

const changedIndex = findChangedPoint(
    pointsX,
    pointsY,
    state.previousX,
    state.previousY
);
const pointWasAdded = pointsX.length > state.previousX.length;
let selected = normalizeSelection(waypoints.selected.indices, pointsX.length);

if (changedIndex >= 0 && (pointWasAdded || selected.length <= 1)) {
    selected = [changedIndex];
    waypoints.selected.indices = selected;
}

const newlySelected = selected.filter(index => !state.previousSelected.includes(index));
const activeIndex = resolveActiveIndex(
    selected,
    newlySelected,
    changedIndex,
    state.activeIndex,
    pointsX.length
);
state.previousX = [...pointsX];
state.previousY = [...pointsY];
state.previousSelected = [...selected];
state.activeIndex = activeIndex;
waypoints._ui_state = state;

if (cb_obj === speed_slider) {
    const pendingSync = speed_slider._pending_sync;
    if (pendingSync !== undefined && speed_slider.value === pendingSync) {
        delete speed_slider._pending_sync;
    } else {
        const targets = selected.length > 0 ? selected : [activeIndex];
        for (const index of targets) {
            if (index >= 0 && index < pointSpeeds.length) pointSpeeds[index] = speed_slider.value;
        }
    }
} else {
    updateSliderSpeed(selected, activeIndex, pointSpeeds);
}

updateSliderTitle(selected, activeIndex, pointSpeeds);
highlight.data = selected.length > 0
    ? { x: selected.map(index => pointsX[index]), y: selected.map(index => pointsY[index]) }
    : activeIndex >= 0
        ? { x: [pointsX[activeIndex]], y: [pointsY[activeIndex]] }
        : { x: [], y: [] };
speeds.data = { speed: pointSpeeds };
labels.data = { x: pointsX, y: pointsY, text: pointSpeeds.map(value => `${Math.round(value)}%`) };

if (pointsX.length === 0) {
    trajectory.data = { xs: [], ys: [], colors: [] };
    stats.text = empty_stats;
    return;
}

const curve = buildTrajectory(robotX, robotY, pointsX, pointsY, pointSpeeds);
trajectory.data = { xs: curve.xs, ys: curve.ys, colors: curve.colors };
smoothing_slider.title = `Trajectory smoothing [${Math.round(smoothing_slider.value)}%]`;
stats.text = `<b>Route</b><br>Waypoints: <b>${pointsX.length}</b><br>Length: <b>${curve.totalDistance.toFixed(2)} m</b>`;
