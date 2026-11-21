"""Test fixtures and validators for OmniDrive laboratory exercises."""

from unittest.mock import Mock

import numpy as np

from .context import f0, l0, r0, robot


def TEST_TASK_1A(wheel_speeds):
    """Weryfikacja poprawności funkcji kinematyki odwrotnej (Zadanie 1a)."""
    cases = [
        ((0.4, 0.2, 0.1), (-0.146, -0.1, 0.546)),
        ((1.0, 0.0, 0.0), (-0.866, 0.0, 0.866)),
        ((-0.3, 0.5, -0.2), (0.310, -0.7, -0.210)),
        ((0.0, 0.0, 1.0), (1.0, 1.0, 1.0)),
        ((0.5, -0.4, 0.3), (-0.333, 0.7, 0.533)),
    ]
    all_ok = True
    for i, ((vx, vy, omega), expected) in enumerate(cases, start=1):
        got = wheel_speeds(vx, vy, omega)
        if np.allclose(got, expected, atol=1e-3):
            print(f"✓ Test {i}: vx={vx}, vy={vy}, omega={omega}")
        else:
            all_ok = False
            print(f"✗ Test {i}: BŁĄD! Dla (vx={vx}, vy={vy}, omega={omega}) otrzymano {got}, oczekiwano {expected}")

    if all_ok:
        print("\n✓ Wszystkie testy zaliczone pomyślnie!")
    return all_ok
