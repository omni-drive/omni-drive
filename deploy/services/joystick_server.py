import argparse
import asyncio
import logging
import logging.handlers
import math

import websockets
from omni_drive import DriveVector, OmniDrive
from websockets.asyncio.server import ServerConnection

from services.generated import remote_pb2

HOST: str = "127.0.0.1"
PORT: int = 8765
PING_INTERVAL_SEC: float = 1.0
PING_TIMEOUT_SEC: float = 2.5

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("JoystickServer")
JOYSTICK_LOGGER_NAME = "omni_drive.joystick_input"
JOYSTICK_LOG_PATH = "/tmp/omni-drive-joystick-input.log"
LOG_FILE_MAX_BYTES = 100 * 1024
LOG_FILE_BACKUP_COUNT = 3

joystick_logger = logging.getLogger(JOYSTICK_LOGGER_NAME)
joystick_logger.setLevel(logging.INFO)
joystick_logger.propagate = False


def setup_joystick_logger(enable_file_logging: bool) -> None:
    if enable_file_logging:
        joystick_handler = logging.handlers.RotatingFileHandler(
            JOYSTICK_LOG_PATH,
            maxBytes=LOG_FILE_MAX_BYTES,
            backupCount=LOG_FILE_BACKUP_COUNT,
        )
        joystick_handler.setFormatter(logging.Formatter("%(asctime)s,%(message)s"))
        joystick_logger.addHandler(joystick_handler)
        logger.info(f"Input file logging enabled -> {JOYSTICK_LOG_PATH}")
    else:
        joystick_logger.addHandler(logging.NullHandler())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Joystick WebSocket Server")
    parser.add_argument(
        "--log-input",
        action="store_true",
        help="Store received frames under /tmp",
    )
    return parser.parse_args()


async def send_telemetry_loop(websocket: ServerConnection, robot: OmniDrive) -> None:
    INTERVAL = 0.1  # 10 Hz

    while True:
        try:
            voltage, _, battery_pct, is_critical = robot.power
            x, y, yaw = robot.pose

            telemetry = remote_pb2.RobotTelemetry(
                battery_voltage=voltage,
                battery_percentage=battery_pct,
                is_battery_critical=is_critical,
                x=x,
                y=y,
                yaw_rad=yaw,
                yaw_deg=math.degrees(yaw),
            )

            await websocket.send(telemetry.SerializeToString())
            await asyncio.sleep(INTERVAL)
        except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
            break
        except Exception as e:
            logger.debug(f"Telemetry streaming error: {e}")
            await asyncio.sleep(INTERVAL)


async def handle_connection(websocket: ServerConnection, robot: OmniDrive) -> None:
    client_addr = websocket.remote_address
    logger.info(f"Client connected from {client_addr}")

    telemetry_task = asyncio.create_task(send_telemetry_loop(websocket, robot))

    try:
        async for message in websocket:
            if not isinstance(message, bytes):
                joystick_logger.error(
                    "client=%s,type=%s,value=%r",
                    client_addr,
                    type(message).__name__,
                    message,
                )
                continue

            try:
                request = remote_pb2.SetSpeedRequest.FromString(message)

                if request.reset_pose:
                    joystick_logger.info("client=%s,action=reset_pose", client_addr)
                    logger.info(f"Reset pose requested from {client_addr}")
                    robot.reset_pose()
                    continue

                joystick_logger.info(
                    "client=%s,type=bytes,x=%.3f,y=%.3f,omega=%.3f",
                    client_addr,
                    request.x,
                    request.y,
                    request.omega,
                )

                velocity = math.hypot(request.x, request.y)
                heading_rad = math.atan2(-request.x, request.y)
                omega = -request.omega

                robot.set_vector(DriveVector(velocity=velocity, heading=heading_rad, omega=omega))
            except Exception as parse_err:
                joystick_logger.warning(
                    "client=%s,type=bytes,size=%d,error=%s",
                    client_addr,
                    len(message),
                    parse_err,
                )
                logger.warning(f"Failed to parse packet from {client_addr}: {parse_err}")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        telemetry_task.cancel()
        robot.set_vector(DriveVector.zero())
        logger.info(f"Client disconnected ({client_addr}) -> Robot halted")


async def main() -> None:
    args = parse_args()
    setup_joystick_logger(args.log_input)

    robot = OmniDrive()

    stop_event = asyncio.Event()

    try:
        async with websockets.serve(
            lambda ws: handle_connection(ws, robot),
            host=HOST,
            port=PORT,
            ping_interval=PING_INTERVAL_SEC,
            ping_timeout=PING_TIMEOUT_SEC,
        ):
            logger.info(f"Joystick server running on ws://{HOST}:{PORT}")
            await stop_event.wait()
    except (asyncio.CancelledError, KeyboardInterrupt):
        logger.info("Stopping WebSocket server...")
    finally:
        robot.close()
        logger.info("Robot closed.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
