import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

import 'config.dart';
import 'generated/remote.pb.dart';
import 'arc_rotation_slider.dart';
import 'custom_joystick.dart';

void main() {
  runApp(
    const MaterialApp(
      debugShowCheckedModeBanner: false,
      home: RemoteControlApp(),
    ),
  );
}

enum ConnectionStatus {
  connecting,
  connected,
  error,
}

class RemoteControlApp extends StatefulWidget {
  const RemoteControlApp({super.key});

  @override
  State<RemoteControlApp> createState() => _RemoteControlAppState();
}

class _RemoteControlAppState extends State<RemoteControlApp> {
  WebSocketChannel? _channel;
  StreamSubscription? _channelSubscription;
  StreamSubscription<GyroscopeEvent>? _gyroscopeSubscription;

  ConnectionStatus _status = ConnectionStatus.connecting;
  RobotTelemetry? _telemetry;

  double _x = 0.0;
  double _y = 0.0;
  double _omega = 0.0;
  double _headingRadians = 0.0;
  DateTime? _lastGyroscopeUpdate;

  Timer? _sendTimer;

  static const Duration _sendInterval = Duration(milliseconds: 50);

  int _zeroStateCount = 0;

  static const double _deadZone = 0.01;
  static const int _zeroStateLimit = 10;
  static const double _gyroDeadband = 0.02; // rad/s (~1.1 deg/s) to filter resting sensor noise

  @override
  void initState() {
    super.initState();

    _initConnection();
    _gyroscopeSubscription = gyroscopeEventStream(
      samplingPeriod: SensorInterval.gameInterval,
    ).listen(_onGyroscopeEvent);

    _sendTimer = Timer.periodic(
      _sendInterval,
      (_) => _sendCurrentState(),
    );
  }

  void _onGyroscopeEvent(GyroscopeEvent event) {
    final now = DateTime.now();
    final previous = _lastGyroscopeUpdate;
    _lastGyroscopeUpdate = now;
    if (previous == null) return;

    final elapsedSeconds = now.difference(previous).inMicroseconds / 1000000.0;
    if (elapsedSeconds <= 0.0 || elapsedSeconds > 0.5) return;

    final z = event.z.abs() > _gyroDeadband ? event.z : 0.0;
    if (z == 0.0) return;

    _headingRadians += z * elapsedSeconds;

    while (_headingRadians > math.pi) {
      _headingRadians -= 2 * math.pi;
    }
    while (_headingRadians < -math.pi) {
      _headingRadians += 2 * math.pi;
    }

    if (mounted) {
      setState(() {});
    }
  }

  Future<void> _initConnection() async {
    if (mounted) {
      setState(() {
        _status = ConnectionStatus.connecting;
      });
    }

    await _channelSubscription?.cancel();
    await _channel?.sink.close();

    try {
      final uri = Uri.parse(
        RobotConfig.wsUrl,
      );

      debugPrint('Connecting to: $uri');
      _channel = WebSocketChannel.connect(uri);

      _channelSubscription = _channel!.stream.listen(
        (data) {
          if (_status != ConnectionStatus.connected && mounted) {
            setState(() {
              _status = ConnectionStatus.connected;
            });
          }

          if (data is List<int>) {
            try {
              final telemetry = RobotTelemetry.fromBuffer(data);
              if (mounted) {
                setState(() {
                  _telemetry = telemetry;
                });
              }
            } catch (e) {
              debugPrint('Error parsing telemetry: $e');
            }
          }
        },
        onError: (error) {
          debugPrint('WebSocket error: $error');
          if (mounted) {
            setState(() {
              _status = ConnectionStatus.error;
            });
          }
        },
        onDone: () {
          debugPrint('WebSocket disconnected');
          if (mounted) {
            setState(() {
              _status = ConnectionStatus.error;
            });
          }
        },
        cancelOnError: true,
      );

      await _channel!.ready.timeout(const Duration(seconds: 2));
      if (mounted) {
        setState(() {
          _status = ConnectionStatus.connected;
        });
      }
    } catch (e) {
      debugPrint('Connection failed: $e');
      if (mounted) {
        setState(() {
          _status = ConnectionStatus.error;
        });
      }
    }
  }

  void _sendCurrentState() {
    if (_status != ConnectionStatus.connected || _channel == null) {
      return;
    }
    final isNearZero = _x.abs() <= _deadZone &&
        _y.abs() <= _deadZone &&
        _omega.abs() <= _deadZone;

    if (isNearZero) {
      _zeroStateCount++;

      if (_zeroStateCount >= _zeroStateLimit) {
        return;
      }
    } else {
      _zeroStateCount = 0;
    }

    final cosHeading = math.cos(_headingRadians);
    final sinHeading = math.sin(_headingRadians);
    final correctedX = _x * cosHeading - _y * sinHeading;
    final correctedY = _x * sinHeading + _y * cosHeading;

    final request = SetSpeedRequest(
      x: correctedX,
      y: correctedY,
      omega: _omega,
    );
    _channel!.sink.add(request.writeToBuffer());
  }

  void _resetPhoneYaw() {
    setState(() {
      _headingRadians = 0.0;
      _lastGyroscopeUpdate = DateTime.now();
    });
    if (!mounted) return;
    ScaffoldMessenger.of(context).clearSnackBars();
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Row(
          children: [
            Icon(Icons.phone_android, color: Colors.blueAccent),
            SizedBox(width: 10),
            Text('Phone heading reset to 0°'),
          ],
        ),
        duration: Duration(milliseconds: 1200),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  void _resetRobotPose() {
    if (_status != ConnectionStatus.connected || _channel == null) {
      ScaffoldMessenger.of(context).clearSnackBars();
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Row(
            children: [
              Icon(Icons.warning_amber_rounded, color: Colors.amberAccent),
              SizedBox(width: 10),
              Text('Robot not connected'),
            ],
          ),
          duration: Duration(milliseconds: 1200),
          behavior: SnackBarBehavior.floating,
        ),
      );
      return;
    }

    final request = SetSpeedRequest(
      resetPose: true,
    );
    _channel!.sink.add(request.writeToBuffer());

    if (!mounted) return;
    ScaffoldMessenger.of(context).clearSnackBars();
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Row(
          children: [
            Icon(Icons.smart_toy, color: Colors.greenAccent),
            SizedBox(width: 10),
            Text('Robot pose reset (X, Y, Yaw -> 0)'),
          ],
        ),
        duration: Duration(milliseconds: 1200),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  void _onJoystickMove(double x, double y) {
    _x = x;
    _y = y;
  }

  void _onJoystickEnd() {
    _x = 0.0;
    _y = 0.0;
    _sendCurrentState();
  }

  void _onOmegaChange(double omega) {
    _omega = omega;
  }

  void _onOmegaEnd() {
    _omega = 0.0;
    _sendCurrentState();
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _channelSubscription?.cancel();
    _gyroscopeSubscription?.cancel();
    _channel?.sink.close();
    super.dispose();
  }

  Widget _buildTelemetryHud() {
    final robotYawDeg = _telemetry?.yawDeg ?? 0.0;
    final robotX = _telemetry?.x ?? 0.0;
    final robotY = _telemetry?.y ?? 0.0;
    final rawDeg = _headingRadians * 180.0 / math.pi;
    final phoneDeg = (rawDeg % 360.0 + 360.0) % 360.0;

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        color: const Color(0xFF1E1E1E),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.white12),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildHudMetric(
                icon: Icons.navigation_outlined,
                iconColor: Colors.blueAccent,
                label: 'Robot Pose',
                value: 'X: ${robotX.toStringAsFixed(2)}m  Y: ${robotY.toStringAsFixed(2)}m',
              ),
              Container(height: 28, width: 1, color: Colors.white12),
              _buildHudMetric(
                icon: Icons.explore,
                iconColor: Colors.greenAccent,
                label: 'Robot Yaw',
                value: '${robotYawDeg.toStringAsFixed(1)}°',
              ),
              Container(height: 28, width: 1, color: Colors.white12),
              _buildHudMetric(
                icon: Icons.phone_android,
                iconColor: Colors.amberAccent,
                label: 'Phone Yaw',
                value: '${phoneDeg.toStringAsFixed(1)}°',
              ),
            ],
          ),
          if (_telemetry != null && _telemetry!.hasTofSensors()) ...[
            const SizedBox(height: 8),
            Container(height: 1, color: Colors.white10),
            const SizedBox(height: 6),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Icon(Icons.radar, size: 14, color: Colors.cyanAccent),
                const SizedBox(width: 6),
                Text(
                  'ToF: L: ${_telemetry!.tofSensors.leftM.toStringAsFixed(2)}m | F: ${_telemetry!.tofSensors.frontM.toStringAsFixed(2)}m | R: ${_telemetry!.tofSensors.rightM.toStringAsFixed(2)}m',
                  style: const TextStyle(fontSize: 11, color: Colors.white70),
                ),
              ],
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildHudMetric({
    required IconData icon,
    required Color iconColor,
    required String label,
    required String value,
  }) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 13, color: iconColor),
            const SizedBox(width: 4),
            Text(
              label,
              style: const TextStyle(
                fontSize: 10,
                color: Colors.white54,
                fontWeight: FontWeight.w500,
              ),
            ),
          ],
        ),
        const SizedBox(height: 2),
        Text(
          value,
          style: const TextStyle(
            fontSize: 12,
            color: Colors.white,
            fontWeight: FontWeight.w700,
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF121212),
      appBar: AppBar(
        title: const Text(
          'Omni Robot Remote',
          style: TextStyle(color: Colors.white, fontSize: 18),
        ),
        backgroundColor: const Color(0xFF1E1E1E),
        centerTitle: false,
        actions: [
          if (_telemetry != null && _telemetry!.batteryVoltage > 0)
            Padding(
              padding: const EdgeInsets.only(right: 16.0),
              child: Center(
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                  decoration: BoxDecoration(
                    color: _telemetry!.isBatteryCritical
                        ? Colors.red.withValues(alpha: 0.25)
                        : const Color(0xFF2A2A2A),
                    borderRadius: BorderRadius.circular(16),
                    border: Border.all(
                      color: _telemetry!.isBatteryCritical
                          ? Colors.redAccent
                          : (_telemetry!.batteryPercentage < 20
                              ? Colors.amberAccent
                              : Colors.greenAccent),
                      width: 1.2,
                    ),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        _telemetry!.isBatteryCritical
                            ? Icons.battery_alert
                            : (_telemetry!.batteryPercentage < 20
                                ? Icons.battery_2_bar
                                : Icons.battery_full),
                        size: 16,
                        color: _telemetry!.isBatteryCritical
                            ? Colors.redAccent
                            : (_telemetry!.batteryPercentage < 20
                                ? Colors.amberAccent
                                : Colors.greenAccent),
                      ),
                      const SizedBox(width: 6),
                      Text(
                        '${_telemetry!.batteryPercentage.toStringAsFixed(0)}% (${_telemetry!.batteryVoltage.toStringAsFixed(1)}V)',
                        style: const TextStyle(
                          fontSize: 12,
                          color: Colors.white,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
        ],
      ),
      bottomNavigationBar: _status == ConnectionStatus.connected
          ? Container(
              color: const Color(0xFF1E1E1E),
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              child: SafeArea(
                child: Row(
                  children: [
                    Expanded(
                      child: ElevatedButton.icon(
                        onPressed: _resetPhoneYaw,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: const Color(0xFF2A2A2A),
                          foregroundColor: Colors.white,
                          padding: const EdgeInsets.symmetric(vertical: 14),
                          elevation: 2,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(10),
                            side: const BorderSide(
                              color: Colors.blueAccent,
                              width: 1.5,
                            ),
                          ),
                        ),
                        icon: const Icon(
                          Icons.phone_android,
                          color: Colors.blueAccent,
                          size: 20,
                        ),
                        label: const Text(
                          'Reset Phone Yaw',
                          style: TextStyle(
                            fontSize: 13,
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                      ),
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: ElevatedButton.icon(
                        onPressed: _resetRobotPose,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: const Color(0xFF2A2A2A),
                          foregroundColor: Colors.white,
                          padding: const EdgeInsets.symmetric(vertical: 14),
                          elevation: 2,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(10),
                            side: const BorderSide(
                              color: Colors.greenAccent,
                              width: 1.5,
                            ),
                          ),
                        ),
                        icon: const Icon(
                          Icons.smart_toy,
                          color: Colors.greenAccent,
                          size: 20,
                        ),
                        label: const Text(
                          'Reset Robot Pose',
                          style: TextStyle(
                            fontSize: 13,
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            )
          : null,
      body: Center(
        child: switch (_status) {
          ConnectionStatus.connecting => const Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                CircularProgressIndicator(color: Colors.blueAccent),
                SizedBox(height: 16),
                Text(
                  'Connecting to robot...',
                  style: TextStyle(color: Colors.white70),
                ),
              ],
            ),
          ConnectionStatus.connected => Column(
              children: [
                _buildTelemetryHud(),
                Expanded(
                  child: Center(
                    child: LayoutBuilder(
                      builder: (context, constraints) {
                        final totalSize =
                            constraints.biggest.shortestSide * 0.90;
                        return SizedBox(
                          width: totalSize,
                          height: totalSize,
                          child: Stack(
                            alignment: Alignment.center,
                            children: [
                              ArcRotationSlider(
                                size: totalSize,
                                onChanged: _onOmegaChange,
                                onEnd: _onOmegaEnd,
                              ),
                              CustomJoystick(
                                size: totalSize * 0.70,
                                onChanged: _onJoystickMove,
                                onEnd: _onJoystickEnd,
                              ),
                            ],
                          ),
                        );
                      },
                    ),
                  ),
                ),
              ],
            ),
          ConnectionStatus.error => Padding(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Icon(Icons.wifi_off_rounded,
                      color: Colors.redAccent, size: 64),
                  const SizedBox(height: 16),
                  const Text(
                    'Failed to connect to the robot server.',
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.white, fontSize: 16),
                  ),
                  const SizedBox(height: 24),
                  ElevatedButton.icon(
                    onPressed: _initConnection,
                    icon: const Icon(Icons.refresh),
                    label: const Text('Retry'),
                  ),
                ],
              ),
            ),
        },
      ),
    );
  }
}
