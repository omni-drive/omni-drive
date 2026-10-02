import os
import random
import socket
import sys
import time

import pytest


def _free_base_port() -> int:
    for _ in range(200):
        base = random.randint(20000, 60000)
        sockets = [socket.socket() for _ in range(3)]
        try:
            for offset, probe in enumerate(sockets):
                probe.bind(("127.0.0.1", base + offset))
            return base
        except OSError:
            continue
        finally:
            for probe in sockets:
                probe.close()
    raise RuntimeError("no three consecutive free TCP ports")


if "omni_drive.ipc" in sys.modules:
    raise pytest.UsageError("omni_drive.ipc was imported before tests/unit set OMNI_DRIVE_IPC")
os.environ["OMNI_DRIVE_IPC"] = f"tcp://127.0.0.1:{_free_base_port()}"


@pytest.fixture(scope="session")
def _simulator():
    from omni_drive.simulator import Simulator

    with Simulator(world="box", seed=0) as sim:
        yield sim


@pytest.fixture
def sim(_simulator):
    _simulator.publish_ultrasonic = False
    _simulator.reset()
    yield _simulator
    _simulator.reset()


@pytest.fixture
def robot(sim, wait_for):
    from omni_drive import OmniDrive

    robot = OmniDrive()
    try:
        assert wait_for(lambda: robot.lidar_scan is not None and robot.odometry is not None)
        time.sleep(0.15)
        yield robot
    finally:
        robot.close()
