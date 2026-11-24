"""Simple unit tests for OmniDrive laboratory exercises."""

import math
from types import SimpleNamespace

import numpy as np
import pandas as pd

from .context import (
    d0_ir_f,
    d0_ir_l,
    d0_ir_r,
    d0_us_f,
    d0_us_l,
    d0_us_r,
    f0,
    l0,
    r0,
    robot,
)


def _run_checks(checks: list[tuple[str, bool, str]]) -> bool:
    all_ok = True
    for i, (label, ok, err_msg) in enumerate(checks, start=1):
        if ok:
            print(f"✓ Test {i}: {label}")
        else:
            all_ok = False
            print(f"✗ Test {i}: BŁĄD! {label} — {err_msg}")
    if all_ok:
        print("\n✓ Wszystkie testy zaliczone pomyślnie!")
    return all_ok


def TEST_TASK_1A(wheel_speeds):
    """Weryfikacja poprawności funkcji kinematyki odwrotnej (Zadanie 1a)."""
    cases = [
        ((0.4, 0.2, 0.1), (-0.146, -0.1, 0.546)),
        ((1.0, 0.0, 0.0), (-0.866, 0.0, 0.866)),
        ((-0.3, 0.5, -0.2), (0.310, -0.7, -0.210)),
        ((0.0, 0.0, 1.0), (1.0, 1.0, 1.0)),
        ((0.5, -0.4, 0.3), (-0.333, 0.7, 0.533)),
    ]
    checks = []
    for (vx, vy, omega), expected in cases:
        got = wheel_speeds(vx, vy, omega)
        ok = np.allclose(got, expected, atol=1e-3)
        checks.append((f"vx={vx}, vy={vy}, omega={omega}", ok, f"otrzymano {got}, oczekiwano {expected}"))
    return _run_checks(checks)


def TEST_TASK_3(lidar_get_x, lidar_get_y):
    """Weryfikacja poprawności funkcji wyznaczania pozycji z LiDAR-u (Zadanie 3)."""
    cases = [
        # ((f0, l0, r0), (front, left, right), (exp_x, exp_y))
        ((1.0, 1.0, 1.0), (1.0, 1.0, 1.0), (0.0, 0.0)),
        ((1.2, 0.8, 0.8), (1.0, 0.8, 0.8), (0.2, 0.0)),
        ((1.0, 1.0, 1.0), (1.0, 0.7, 1.3), (0.0, 0.3)),
        ((0.6, 1.4, 0.5), (0.4, 1.1, 0.8), (0.2, 0.3)),
    ]

    orig_robot, orig_f0, orig_l0, orig_r0 = robot.get(), f0.get(), l0.get(), r0.get()
    fake_lidar = SimpleNamespace(beam_front=0.0, beam_left=0.0, beam_right=0.0)
    robot.set(SimpleNamespace(lidar=fake_lidar))

    checks = []
    try:
        for (f0_v, l0_v, r0_v), (front, left, right), expected in cases:
            f0.set(f0_v)
            l0.set(l0_v)
            r0.set(r0_v)
            fake_lidar.beam_front = front
            fake_lidar.beam_left = left
            fake_lidar.beam_right = right

            got = (lidar_get_x(), lidar_get_y())
            ok = np.allclose(got, expected, atol=1e-3)
            checks.append((f"x={expected[0]:.1f}, y={expected[1]:.1f}", ok, f"otrzymano {got}, oczekiwano {expected}"))
    finally:
        robot.set(orig_robot)
        f0.set(orig_f0)
        l0.set(orig_l0)
        r0.set(orig_r0)

    return _run_checks(checks)


def TEST_TASK_3_Q(Q, final_drift, N_steps):
    """Weryfikacja obliczenia wariancji szumu procesu Q (Zadanie 3)."""
    expected = final_drift**2 / N_steps
    sqrt_q_mm = math.sqrt(Q) * 1000.0
    checks = [
        ("Q == final_drift² / N_steps", np.isclose(Q, expected, rtol=1e-6), f"otrzymano {Q:.6f}, oczekiwano {expected:.6f}"),
        ("√Q w rozsądnym zakresie (0.1–20 mm/krok)", 0.1 <= sqrt_q_mm <= 20.0, f"otrzymano √Q = {sqrt_q_mm:.2f} mm"),
    ]
    return _run_checks(checks)


def TEST_TASK_4(calc_R):
    """Weryfikacja poprawności odpornej estymacji szumu pomiaru MAD (Zadanie 4)."""
    expected_10mm = (1.4826 * 0.01) ** 2
    cases = [
        ("czysty błąd ±10 mm", pd.Series([-0.01, -0.01, 0.0, 0.01, 0.01]), expected_10mm),
        ("błąd ±10 mm ze stałym offsetem +50 mm", pd.Series([0.04, 0.04, 0.05, 0.06, 0.06]), expected_10mm),
        (
            "błąd ±10 mm z outlierami i NaN",
            pd.Series([-0.01, -0.01, -0.01, 0.0, 0.0, 0.01, 0.01, 0.01, 0.5, -0.4, np.nan]),
            expected_10mm,
        ),
    ]
    checks = []
    for label, series, expected in cases:
        got = float(calc_R(series))
        ok = np.isclose(got, expected, rtol=1e-2)
        checks.append((label, ok, f"otrzymano R={got:.6f}, oczekiwano {expected:.6f}"))
    return _run_checks(checks)


def TEST_TASK_5_1(kalman_predict, kalman_update):
    """Weryfikacja kroków predykcji i korekty Filtru Kalmana 1D (Zadanie 5.1)."""
    cases = [
        ("predykcja: ruch +x", kalman_predict, (0.10, 0.004, 0.02, 0.001), (0.12, 0.005)),
        ("predykcja: ruch -x", kalman_predict, (0.25, 0.002, -0.05, 0.003), (0.20, 0.005)),
        ("korekta: P = R (K = 0.5)", kalman_update, (0.12, 0.005, 0.15, 0.005), (0.135, 0.0025)),
        ("korekta: K = 0.25, ujemna innowacja", kalman_update, (0.12, 0.004, 0.10, 0.012), (0.115, 0.003)),
        ("korekta: R ≪ P (K = 0.9)", kalman_update, (0.00, 0.009, 0.20, 0.001), (0.18, 0.0009)),
    ]
    checks = []
    for label, fn, args, expected in cases:
        got = fn(*args)
        ok = np.allclose(got, expected, atol=1e-6)
        checks.append((label, ok, f"otrzymano {got}, oczekiwano {expected}"))
    return _run_checks(checks)


def TEST_TASK_6(z, Q_mat, R_mat, H, Q, R_ir, R_us):
    """Weryfikacja wektora z oraz macierzy Q_mat, R_mat i H dla Filtru Kalmana 2D (Zadanie 6)."""
    expected_h = np.array([[1, 0], [0, 1], [0, -1], [1, 0], [0, 1], [0, -1]])
    matrix_cases = [
        ("Q_mat", Q_mat, np.diag([Q, Q])),
        ("R_mat", R_mat, np.diag([R_ir, R_ir, R_ir, R_us, R_us, R_us])),
        ("H", H, expected_h),
    ]
    checks = []
    for name, got, expected in matrix_cases:
        arr = np.asarray(got, dtype=float)
        ok = arr.shape == expected.shape and np.allclose(arr, expected, atol=1e-12)
        checks.append((f"macierz {name} {expected.shape}", ok, f"otrzymano:\n{arr}"))

    dx, dy = 0.05, 0.03
    d0_vars = (d0_ir_f, d0_ir_l, d0_ir_r, d0_us_f, d0_us_l, d0_us_r)
    d0_test = (0.40, 0.50, 0.30, 0.42, 0.52, 0.32)
    orig_robot = robot.get()
    orig_d0 = [v.get() for v in d0_vars]

    fake_robot = SimpleNamespace(
        ir_front=SimpleNamespace(distance=d0_test[0] - dx),
        ir_left=SimpleNamespace(distance=d0_test[1] - dy),
        ir_right=SimpleNamespace(distance=d0_test[2] + dy),
        us_front=SimpleNamespace(distance=d0_test[3] - dx),
        us_left=SimpleNamespace(distance=d0_test[4] - dy),
        us_right=SimpleNamespace(distance=d0_test[5] + dy),
    )
    try:
        robot.set(fake_robot)
        for var, val in zip(d0_vars, d0_test):
            var.set(val)
        z_vals = [float(fn()) for fn in z]
    finally:
        robot.set(orig_robot)
        for var, val in zip(d0_vars, orig_d0):
            var.set(val)

    expected_z = expected_h @ np.array([dx, dy])
    sensor_names = ("IR przód", "IR lewy", "IR prawy", "US przód", "US lewy", "US prawy")
    for name, got, exp in zip(sensor_names, z_vals, expected_z):
        ok = np.isclose(got, exp, atol=1e-6)
        checks.append((f"z[{name}] = {exp * 100:+.1f} cm", ok, f"otrzymano {got * 100:+.1f} cm (sprawdź d0 i znak)"))

    return _run_checks(checks)
