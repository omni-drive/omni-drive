"""Testy do zadań z notebooka. Każda funkcja wypisuje ✓/✗ dla kolejnych sprawdzeń.

Testy funkcji nie poruszają robotem: tam, gdzie funkcja korzysta z robota, na czas testu
podstawiamy w przestrzeni nazw notebooka atrapę (fake), a potem przywracamy oryginał.
"""

import math
import types
from contextlib import contextmanager

import numpy as np
import pandas as pd
from omni_drive import Angle, Distance, DriveVector, Time


def _report(checks):
    """checks: lista (opis, ok[, komunikat przy błędzie])."""
    all_ok = True
    for i, (name, ok, *hint) in enumerate(checks, start=1):
        ok = bool(ok)
        all_ok &= ok
        extra = "" if ok else " – BŁĄD!" + (f" {hint[0]}" if hint and hint[0] else "")
        print(f"{'✓' if ok else '✗'} Test {i}: {name}{extra}")
    if all_ok and len(checks) > 1:
        print("\n✓ Wszystkie testy zaliczone pomyślnie!")
    return all_ok


@contextmanager
def _patched(ns, **values):
    """Tymczasowo podmienia zmienne w przestrzeni nazw notebooka (ns = fn.__globals__)."""
    missing = object()
    saved = {k: ns.get(k, missing) for k in values}
    ns.update(values)
    try:
        yield
    finally:
        for k, v in saved.items():
            if v is missing:
                ns.pop(k, None)
            else:
                ns[k] = v


def _close(a, b, tol):
    try:
        return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=tol)
    except (TypeError, ValueError):
        return False


def test_wheel_speeds(wheel_speeds):
    checks = []
    for (vx, vy, omega), expected in [
        ((0.4, 0.2, 0.1), (0.546, -0.1, -0.146)),
        ((1.0, 0.0, 0.0), (0.866, 0.0, -0.866)),
        ((-0.3, 0.5, -0.2), (-0.210, -0.7, 0.310)),
        ((0.0, 0.0, 1.0), (1.0, 1.0, 1.0)),
        ((0.5, -0.4, 0.3), (0.533, 0.7, -0.333)),
    ]:
        got = wheel_speeds(vx, vy, omega)
        checks.append((f"vx={vx}, vy={vy}, omega={omega}", _close(got, expected, 1e-3),
                       f"otrzymano {got}, oczekiwano {expected}"))
    return _report(checks)


class _FakeContextRobot:
    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        pass

    def stop(self):
        pass


async def test_drive_raw(drive_raw, wheel_speeds):
    captured = []
    checks = []
    with _patched(drive_raw.__globals__, robot=_FakeContextRobot(),
                  set_wheels=lambda w1, w2, w3: captured.append((w1, w2, w3))):
        for v, deg, omega in [(0.5, 0, 0.0), (0.5, 90, 0.0), (0.3, 180, 0.2), (0.4, -45, 0.0)]:
            captured.clear()
            await drive_raw(Time.ms(50), DriveVector(velocity=v, heading=Angle.deg(deg), omega=omega))
            heading = math.radians(deg)
            expected = wheel_speeds(v * math.cos(heading), v * math.sin(heading), omega)
            got = captured[0] if captured else None
            checks.append((f"velocity={v}, heading={deg}°, omega={omega}",
                           got is not None and _close(got, expected, 1e-6),
                           f"otrzymano {got}, oczekiwano {expected}. Sprawdź cos/sin w vx, vy."))
    return _report(checks)


class _FakeDriveRobot:
    def __init__(self):
        self.calls = []

    async def drive_for(self, duration, vector):
        self.calls.append((float(duration), vector.heading.degrees % 360, vector.velocity))

    async def pause_for(self, duration):
        pass


async def test_burst(burst):
    ns = burst.__globals__
    fake = _FakeDriveRobot()
    with _patched(ns, robot=fake):
        await burst(Angle.deg(90), reps=2, leg=Distance.cm(10), velocity=0.8)
    calls = fake.calls
    d = float(Distance.cm(10) / (0.8 * ns["V_MAX"]))
    expected = [(d, 90.0, 0.8), (d, 270.0, 0.8)] * 2
    same_len = len(calls) == len(expected)
    got = str([(round(c[0], 3), round(c[1]), c[2]) for c in calls])
    return _report([
        ("liczba przejazdów (2 × tam i z powrotem)", same_len, f"otrzymano {len(calls)} przejazdy"),
        ("czas przejazdu odcinka = leg / (velocity * V_MAX)",
         same_len and all(abs(c[0] - e[0]) < 1e-6 for c, e in zip(calls, expected)), got),
        ("kierunki: 90°, 270°, 90°, 270°",
         same_len and all(abs(c[1] - e[1]) < 1e-6 for c, e in zip(calls, expected)), got),
        ("prędkość = velocity", same_len and all(abs(c[2] - e[2]) < 1e-9 for c, e in zip(calls, expected)), got),
    ])


def test_lidar_position(lidar_get_x, lidar_get_y):
    checks = []
    for (front, left, right), (ex, ey), desc in [
        ((0.30, 0.35, 0.25), (0.10, 0.05), "10 cm do przodu, 5 cm w prawo"),
        ((0.45, 0.27, 0.33), (-0.05, -0.03), "5 cm do tyłu, 3 cm w lewo"),
    ]:
        beams = {k: Distance.m(v) for k, v in (("beam_front", front), ("beam_left", left), ("beam_right", right))}
        fake = types.SimpleNamespace(lidar=types.SimpleNamespace(**beams))
        start = dict(f0=Distance.m(0.40), l0=Distance.m(0.30), r0=Distance.m(0.30))
        with _patched(lidar_get_x.__globals__, robot=fake, **start):
            gx, gy = lidar_get_x(), lidar_get_y()
        checks.append((desc, _close((gx, gy), (ex, ey), 1e-9),
                       f"otrzymano x={gx}, y={gy}, oczekiwano x={ex}, y={ey}"))
    checks.append(("wynik to Distance (nie zwykła liczba)",
                   isinstance(gx, Distance) and isinstance(gy, Distance),
                   "zwróć różnicę odległości bez .meters"))
    return _report(checks)


def test_Q(Q, final_drift, N_steps):
    return _report([
        ("Q = final_drift² / N_steps", _close(Q, float(final_drift) ** 2 / N_steps, 1e-12)),
        ("√Q jest rozsądne (od 0,1 do 20 mm na krok)", 1e-4 <= math.sqrt(Q) <= 0.02, f"√Q = {math.sqrt(Q) * 1000:.2f} mm"),
    ])


def test_calc_R(calc_R):
    rng = np.random.default_rng(0)
    e = pd.Series(np.r_[rng.normal(0, 0.005, 300), [0.5, -0.4, 0.8, np.nan]])
    got = math.sqrt(calc_R(e)) * 1000
    return _report([
        (f"szum 5 mm z fałszywymi echami → √R = {got:.1f} mm", 4.0 <= got <= 6.0,
         "oczekiwano ok. 5 mm (czy używasz mediany, a nie średniej?)"),
    ])


def test_R(R_ir, R_us, still, moving, calc_R):
    return _report([
        ("R_ir z fazy spoczynku (still)", _close(R_ir, calc_R(still["infrared"] - still["lidar"]), 1e-15)),
        ("R_us z fazy ruchu (moving)", _close(R_us, calc_R(moving["ultrasonic"] - moving["lidar"]), 1e-15)),
    ])


def test_kalman(kalman_predict, kalman_update):
    checks = []
    for fn, args, expected, desc in [
        (kalman_predict, (Distance.m(0.10), 0.004, Distance.m(0.02), 0.001), (0.12, 0.005), "predykcja: x + dx, P + Q"),
        (kalman_update, (Distance.m(0.12), 0.005, Distance.m(0.15), 0.005), (0.135, 0.0025), "korekta: P = R → K = 0,5"),
        (kalman_update, (Distance.m(0.12), 0.004, Distance.m(0.10), 0.012), (0.115, 0.003), "korekta: K = 0,25"),
    ]:
        got = fn(*args)
        is_distance = isinstance(got, tuple) and len(got) == 2 and isinstance(got[0], Distance)
        checks.append((desc, is_distance and _close(got, expected, 1e-9),
                       f"otrzymano {got}, oczekiwano {expected}" + ("" if is_distance else "; x musi być Distance")))
    return _report(checks)


def test_rmse(rmse):
    got = rmse(pd.Series([0.10, 0.20, 0.40, np.nan]), pd.Series([0.10, 0.10, 0.10, 0.10]))
    expected = math.sqrt((0.0 ** 2 + 0.1 ** 2 + 0.3 ** 2) / 3)
    return _report([(f"RMSE = {got:.4f} m", _close(got, expected, 1e-12), f"oczekiwano {expected:.4f} m")])


def test_task_5_2(df, rmse):
    r_odo, r_ir, r_kf = (rmse(df[n], df["lidar"]) for n in ("odometry", "infrared", "kalman"))
    hint = "Sprawdź dx (od poprzedniego kroku) i znak w z = d0 - d."
    return _report([
        ("filtr dokładniejszy od odometrii", r_kf < r_odo, hint),
        ("filtr dokładniejszy od samego czujnika IR", r_kf < r_ir, hint),
    ])


def test_task_5_3(df, rmse):
    r_odo, r_ir, r_fu = (rmse(df[n], df["lidar"]) for n in ("odometry", "kalman_ir", "kalman_fusion"))
    return _report([
        ("fuzja dokładniejsza od odometrii", r_fu < r_odo),
        ("fuzja nie gorsza od samego IR (tolerancja 2 mm)", r_fu <= r_ir + 0.002),
        ("pomiar US użyty w pętli (z_us ma wartości)",
         df["ultrasonic"].notna().sum() > 10 and df["ultrasonic"].abs().sum() > 0),
    ])


def test_task_6_model(z, Q_mat, R_mat, H, Q, R_ir, R_us):
    dx, dy, d0 = 0.05, 0.03, 0.40

    def s(d):
        return types.SimpleNamespace(distance=Distance.m(d))

    fake = types.SimpleNamespace(ir_front=s(d0 - dx), ir_left=s(d0 + dy), ir_right=s(d0 - dy),
                                 us_front=s(d0 - dx), us_left=s(d0 + dy), us_right=s(d0 - dy))
    baselines = dict.fromkeys(("d0_ir_f", "d0_ir_l", "d0_ir_r", "d0_us_f", "d0_us_l", "d0_us_r"), Distance.m(d0))
    with _patched(z[0].__globals__, robot=fake, **baselines):
        z_vals = np.array([float(f()) for f in z], dtype=float)

    H_arr = np.asarray(H, dtype=float)
    checks = [
        ("Q_mat to diag([Q, Q])", np.shape(Q_mat) == (2, 2) and _close(Q_mat, np.diag([Q, Q]), 1e-15)),
        ("R_mat ma R_ir dla IR i R_us dla US na przekątnej",
         np.shape(R_mat) == (6, 6) and _close(R_mat, np.diag([R_ir] * 3 + [R_us] * 3), 1e-15)),
        ("H ma kształt 6 × 2", H_arr.shape == (6, 2)),
    ]
    if H_arr.shape == (6, 2):
        pred = H_arr @ np.array([dx, dy])
        names = ["IR przód", "IR lewy", "IR prawy", "US przód", "US lewy", "US prawy"]
        for k, n in enumerate(names):
            checks.append((f"{n}: z = {z_vals[k] * 100:+.1f} cm, H·[x, y] = {pred[k] * 100:+.1f} cm",
                           abs(pred[k] - z_vals[k]) < 1e-9, "sprawdź znak w z albo w wierszu H"))
    return _report(checks)


def test_task_6(rec, rmse_2d):
    r_odo, r_kf = (rmse_2d(rec.data[n], rec.data["lidar"]) for n in ("odometry", "kalman"))
    return _report([("filtr 2D dokładniejszy od odometrii", r_kf < r_odo, "Sprawdź znaki w z i H (test powyżej).")])
