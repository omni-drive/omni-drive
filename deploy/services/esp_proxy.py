import logging
import threading
import time

import serial
import zmq
from cobs import cobs
from omni_drive.transport import IPC

from services.ports import find_esp_port, open_serial

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("EspProxy")


def telemetry_loop(serial_port: serial.Serial, pub: zmq.Socket, stop_event: threading.Event) -> None:
    """Receives COBS frames from UART and publishes raw Protobuf to ZMQ."""
    buf = bytearray()
    synchronized = False

    while not stop_event.is_set():
        try:
            chunk = serial_port.read(max(1, serial_port.in_waiting))
        except (serial.SerialException, TypeError, OSError):
            break

        if not chunk:
            continue
        buf.extend(chunk)

        while b"\x00" in buf:
            zero_idx = buf.find(b"\x00")
            packet = bytes(buf[:zero_idx])
            del buf[: zero_idx + 1]

            if not synchronized:
                synchronized = True
                continue

            if not packet:
                continue

            try:
                raw_protobuf = cobs.decode(packet)
                pub.send(raw_protobuf)
            except cobs.DecodeError:
                logger.warning(
                    "COBS Decode Error! Raw packet len=%d hex=%s ascii=%r",
                    len(packet),
                    packet.hex(" "),
                    packet.decode("ascii", errors="replace"),
                )
                continue
            except zmq.ZMQError:
                return


def command_loop(serial_port: serial.Serial, pull: zmq.Socket, stop_event: threading.Event) -> None:
    """Receives commands from ZMQ with timeout and sends COBS frames to UART."""
    while not stop_event.is_set():
        try:
            protobuf_data = pull.recv()
        except zmq.Again:
            continue
        except zmq.ZMQError:
            return

        try:
            frame = cobs.encode(protobuf_data) + b"\x00"
            serial_port.write(frame)
            serial_port.flush()
        except (serial.SerialException, OSError) as e:
            logger.warning("UART write error (%s).", e)
            break


def main() -> None:
    ctx = zmq.Context.instance()

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 1)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(IPC.ESP_TELEMETRY)

    pull = ctx.socket(zmq.PULL)
    pull.setsockopt(zmq.RCVTIMEO, 200)
    pull.setsockopt(zmq.LINGER, 0)
    pull.bind(IPC.ESP_COMMANDS)

    logger.info("Service started. PUB: %s | PULL: %s", IPC.ESP_TELEMETRY, IPC.ESP_COMMANDS)

    try:
        while True:
            port = find_esp_port()

            if not port:
                logger.info("Waiting for ESP32...")
                time.sleep(1.0)
                continue

            try:
                logger.info("Connecting to %s...", port)

                with open_serial(url=port, baudrate=921600) as serial_port:
                    logger.info("Connected to %s.", port)
                    stop_event = threading.Event()

                    t_rx = threading.Thread(
                        target=telemetry_loop,
                        args=(serial_port, pub, stop_event),
                        name="EspProxy-Rx",
                        daemon=True,
                    )
                    t_tx = threading.Thread(
                        target=command_loop,
                        args=(serial_port, pull, stop_event),
                        name="EspProxy-Tx",
                        daemon=True,
                    )

                    t_rx.start()
                    t_tx.start()

                    while t_rx.is_alive() and t_tx.is_alive():
                        time.sleep(0.1)

                    stop_event.set()
                    try:
                        serial_port.close()
                    except Exception:
                        pass

                    t_rx.join(timeout=1.0)
                    t_tx.join(timeout=1.0)

            except Exception as e:
                logger.warning("Connection lost (%s). Retrying in 1s...", e)
                time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("Stopping proxy service...")
    finally:
        pub.close(linger=0)
        pull.close(linger=0)
        ctx.term()


if __name__ == "__main__":
    main()
