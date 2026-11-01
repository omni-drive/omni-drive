import time

from omni_drive import OmniDrive


def main() -> None:
    print("Connecting to robot...")
    robot = OmniDrive()
    print("Connected! Streaming telemetry (Press Ctrl+C to exit)...\n")

    try:
        while True:
            robot.print_status()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nConnection test ended.")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
