import logging
import math
import time

import serial
import zmq
from omni_drive.transport import IPC, LidarScan, Point2D

from services.ports import find_hokuyo_port, open_serial

START_STEP: int = 44
TOTAL_STEPS: int = 1024
FRONT_STEP_OFFSET: int = 384
RAD_PER_STEP: float = 2.0 * math.pi / TOTAL_STEPS

TRIG_LUT: list[tuple[float, float]] = [
    (
        math.cos((START_STEP + i - FRONT_STEP_OFFSET) * RAD_PER_STEP),
        math.sin((START_STEP + i - FRONT_STEP_OFFSET) * RAD_PER_STEP),
    )
    for i in range(TOTAL_STEPS)
]

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("LidarProxy")


def init_lidar(serial_port: serial.Serial) -> None:
    """Sends the SCIP 2.0 configuration sequence to the Hokuyo."""
    time.sleep(0.1)
    serial_port.write(b"QT\n")
    serial_port.flush()
    time.sleep(0.1)

    serial_port.write(b"SCIP2.0\n")
    serial_port.flush()
    time.sleep(0.05)

    serial_port.write(b"BM\n")
    serial_port.flush()
    time.sleep(0.1)

    serial_port.write(b"MD0044072501000\n")
    serial_port.flush()
    time.sleep(0.1)


def parse_points(data: bytes) -> list[Point2D]:
    """Converts 3-byte SCIP 2.0 raw bytes to Cartesian coordinates using precomputed LUT."""
    points: list[Point2D] = []
    step_idx = 0
    limit = len(data) - 2

    for i in range(0, limit, 3):
        dist = ((data[i] - 0x30) << 12) + ((data[i + 1] - 0x30) << 6) + (data[i + 2] - 0x30)

        if 20 <= dist <= 5600 and step_idx < len(TRIG_LUT):
            cos_val, sin_val = TRIG_LUT[step_idx]
            points.append(
                (
                    round(dist * cos_val, 1),
                    round(dist * sin_val, 1),
                )
            )
        step_idx += 1

    return points


class FastScipReader:
    def __init__(self, serial_port: serial.Serial):
        self.ser = serial_port
        self.buf = bytearray()

    def fetch_scan_frame(self) -> list[Point2D]:
        lines: list[bytes] = []

        while True:
            nl_pos = self.buf.find(b"\n")
            if nl_pos != -1:
                line = bytes(self.buf[:nl_pos]).rstrip(b"\r")
                del self.buf[: nl_pos + 1]

                if not line:
                    if lines:
                        break
                    continue
                lines.append(line)
            else:
                try:
                    chunk = self.ser.read(max(1, self.ser.in_waiting))
                except (serial.SerialException, OSError) as e:
                    raise ConnectionResetError(f"Serial read error: {e}") from e

                if not chunk:
                    raise ConnectionResetError("Serial stream closed (timeout/EOF)")
                self.buf.extend(chunk)

        if len(lines) < 4 or lines[1][:2] not in (b"00", b"99"):
            return []

        raw_data = b"".join(line[:-1] for line in lines[3:])
        return parse_points(raw_data)


def lidar_loop(serial_port: serial.Serial, pub: zmq.Socket) -> None:
    init_lidar(serial_port)
    logger.info("LiDAR initialized. Publishing scans.")
    reader = FastScipReader(serial_port)

    while True:
        pts = reader.fetch_scan_frame()
        if pts:
            scan = LidarScan(points=pts, timestamp=time.monotonic())
            pub.send(scan.to_bytes())


def main() -> None:
    ctx = zmq.Context.instance()

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 1)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(IPC.LIDAR_SCANS)

    logger.info("Service started. PUB: %s", IPC.LIDAR_SCANS)

    try:
        while True:
            port = find_hokuyo_port()

            if not port:
                logger.info("Waiting for Hokuyo LiDAR USB port...")
                time.sleep(1.0)
                continue

            try:
                logger.info("Connecting to %s...", port)

                with open_serial(url=port, baudrate=115200) as serial_port:
                    lidar_loop(serial_port, pub)

            except (ConnectionResetError, serial.SerialException, OSError) as e:
                logger.warning("Connection lost (%s). Retrying in 1s...", e)
                time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("Stopping LiDAR proxy service...")
    finally:
        pub.close(linger=0)
        ctx.term()


if __name__ == "__main__":
    main()
