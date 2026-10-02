delete waypoints._drag;
delete waypoints._ui_state;
delete speed_slider._pending_sync;

waypoints.data = { x: [], y: [] };
speeds.data = { speed: [] };
labels.data = { x: [], y: [], text: [] };
trajectory.data = { xs: [], ys: [], colors: [] };
highlight.data = { x: [], y: [] };
waypoints.selected.indices = [];
stats.text = empty_stats;
