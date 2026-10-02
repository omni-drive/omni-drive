const drag = waypoints._drag;
const index = drag?.index ?? -1;
const x = cb_obj.x;
const y = cb_obj.y;
const xs = Array.from(drag?.originX || waypoints.data.x || []);
const ys = Array.from(drag?.originY || waypoints.data.y || []);

if (index >= 0 && index < xs.length && index < ys.length && x != null && y != null) {
    if (drag?.selection?.length > 1) {
        waypoints.selected.indices = [...drag.selection];
        const dx = x - drag.start.x;
        const dy = y - drag.start.y;
        for (const selectedIndex of drag.selection) {
            if (selectedIndex >= 0 && selectedIndex < xs.length && selectedIndex < ys.length) {
                xs[selectedIndex] += dx;
                ys[selectedIndex] += dy;
            }
        }
    } else {
        xs[index] = x;
        ys[index] = y;
    }
    waypoints.data = { x: xs, y: ys };
}
