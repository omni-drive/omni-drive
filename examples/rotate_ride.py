import time

from omni_drive import Angle, DriveVector, OmniDrive

CONTROL_INTERVAL_S = 0.05
LOG_INTERVAL_S = 0.1
SIDE_DURATION_S = 3.0
STOP_DURATION_S = 0.7
DRIVE_SPEED = 0.9

RETURN_SPEED = 0.45
RETURN_OMEGA = 0.45
RETURN_DURATION_S = 7.0


def drive_for(
    robot: OmniDrive,
    vector: DriveVector,
    duration_s: float,
) -> None:
    robot.set_vector(vector)
    end_time = time.monotonic() + duration_s
    next_log_time = time.monotonic()

    try:
        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_log_time:
                robot.print_status()
                next_log_time = now + LOG_INTERVAL_S
            time.sleep(CONTROL_INTERVAL_S)
    finally:
        robot.set_vector(DriveVector.zero())


def pause_for(robot: OmniDrive, duration_s: float) -> None:
    drive_for(robot, DriveVector.zero(), duration_s)


def run_diagnostic_test() -> None:
    robot = OmniDrive()

    try:
        time.sleep(0.2)
        robot.reset_pose()

        print("\n=== STARTING OUT-AND-BACK TEST ===")
        print(f"Leg 1: Speed {DRIVE_SPEED} | Duration {SIDE_DURATION_S}s")
        print(f"Leg 2: Speed {RETURN_SPEED} | Omega {RETURN_OMEGA} | Duration {RETURN_DURATION_S}s")

        print("\n[Drive 1/2] Straight out, heading 0°, omega 0.0")
        drive_for(
            robot,
            DriveVector(
                velocity=DRIVE_SPEED,
                heading=Angle.deg(0),
                omega=0.0,
            ),
            duration_s=SIDE_DURATION_S,
        )
        pause_for(robot, duration_s=STOP_DURATION_S)

        print(f"\n[Drive 2/2] Returning, heading 180°, omega {RETURN_OMEGA}")
        drive_for(
            robot,
            DriveVector(
                velocity=RETURN_SPEED,
                heading=Angle.deg(180),
                omega=RETURN_OMEGA,
            ),
            duration_s=RETURN_DURATION_S,
        )
        pause_for(robot, duration_s=STOP_DURATION_S)

    except KeyboardInterrupt:
        print("\nDrive interrupted by user.")
    finally:
        print("\n=== OUT-AND-BACK TEST RESULT ===")
        print("Check the robot's path, final position, and yaw stability.")
        robot.close()


if __name__ == "__main__":
    run_diagnostic_test()
