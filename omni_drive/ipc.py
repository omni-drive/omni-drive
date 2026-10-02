import os
from enum import StrEnum

import zmq

TCP_BASE_ENV_VAR = "OMNI_DRIVE_IPC"


def _endpoint(unix_socket: str, port_offset: int) -> str:
    base = os.environ.get(TCP_BASE_ENV_VAR, "").strip()
    if not base:
        return unix_socket
    prefix, separator, port = base.rpartition(":")
    if not separator or not port.isdigit() or "://" not in prefix:
        raise ValueError(f"{TCP_BASE_ENV_VAR} must look like tcp://127.0.0.1:5650, got {base!r}")
    return f"{prefix}:{int(port) + port_offset}"


class IPC(StrEnum):
    ESP_TELEMETRY = _endpoint("ipc:///tmp/esp_telemetry.ipc", 0)
    ESP_COMMANDS = _endpoint("ipc:///tmp/esp_commands.ipc", 1)
    LIDAR_SCANS = _endpoint("ipc:///tmp/lidar_scans.ipc", 2)


def bind_publisher(context: zmq.Context, address: str) -> zmq.Socket:
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(address)
    return socket
