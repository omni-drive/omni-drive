const x = cb_obj.x;
const y = cb_obj.y;
const xs = waypoints.data.x || [];
const ys = waypoints.data.y || [];
const xTolerance = grab_radius_px * (x_range.end - x_range.start) / plot_size;
const yTolerance = grab_radius_px * (y_range.end - y_range.start) / plot_size;
let nearest = -1;
let nearestDistance = Infinity;

for (let i = 0; i < Math.min(xs.length, ys.length); i++) {
    const dx = (xs[i] - x) / xTolerance;
    const dy = (ys[i] - y) / yTolerance;
    const distance = dx * dx + dy * dy;
    if (distance <= 1 && distance < nearestDistance) {
        nearest = i;
        nearestDistance = distance;
    }
}

waypoints._drag = {
    index: nearest,
    selection: Array.from(waypoints.selected.indices || []),
    start: { x, y },
    originX: [...xs],
    originY: [...ys],
};
