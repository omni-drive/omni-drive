import os
from contextlib import contextmanager

import serial
from serial.tools import list_ports


@contextmanager
def open_serial(url: str, baudrate: int):
    serial_port = serial.Serial()
    serial_port.port = url
    serial_port.baudrate = baudrate
    serial_port.open()

    serial_port.reset_input_buffer()

    try:
        yield serial_port
    finally:
        if serial_port.is_open:
            serial_port.close()


def find_hokuyo_port() -> str | None:
    """Finds the USB port of the Hokuyo LiDAR or returns None."""
    HOKUYO_VID = 0x15D1

    for port in list_ports.comports():
        if port.vid == HOKUYO_VID or "hokuyo" in (port.description or "").lower():
            return port.device
    return None


def find_esp_port() -> str | None:
    """Finds the serial port of the ESP32.

    Priority:
    1. /dev/ttyAMA0 (hardware UART on Raspberry Pi)
    2. USB device matching known ESP32 / USB-UART bridge VIDs
    3. Generic fallback to /dev/ttyUSB0 or /dev/ttyACM0
    """
    if os.path.exists("/dev/ttyAMA0"):
        return "/dev/ttyAMA0"

    ESP_VIDS = {0x10C4, 0x1A86, 0x303A, 0x0403}
    for port in list_ports.comports():
        if port.vid in ESP_VIDS:
            return port.device

    for fallback in ["/dev/ttyUSB0", "/dev/ttyACM0"]:
        if os.path.exists(fallback):
            return fallback

    return None
