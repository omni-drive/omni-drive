from omni_drive.generated import robot_pb2
from omni_drive.ipc import IPC
from omni_drive.messages import LidarScan
from omni_drive.zmq_channel import ZmqPublisher, ZmqSubscriber


def get_esp_telemetry_subscriber() -> ZmqSubscriber[robot_pb2.TelemetryResponse]:
    return ZmqSubscriber(IPC.ESP_TELEMETRY, robot_pb2.TelemetryResponse.FromString)


def get_esp_command_publisher() -> ZmqPublisher[robot_pb2.VelocityCommand]:
    return ZmqPublisher(IPC.ESP_COMMANDS, robot_pb2.VelocityCommand.SerializeToString)


def get_lidar_subscriber() -> ZmqSubscriber[LidarScan]:
    return ZmqSubscriber(IPC.LIDAR_SCANS, LidarScan.from_bytes)
