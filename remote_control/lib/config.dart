import 'package:flutter/foundation.dart';

class RobotConfig {
  static String get host =>
      kIsWeb && Uri.base.host.isNotEmpty ? Uri.base.host : '127.0.0.1';

  static String get wsUrl {
    if (kIsWeb && Uri.base.host.isNotEmpty) {
      if (Uri.base.port == 80 || Uri.base.port == 443 || !Uri.base.hasPort) {
        return 'ws://${Uri.base.host}/ws';
      }
      return 'ws://${Uri.base.host}:8765';
    }
    return 'ws://127.0.0.1:8765';
  }
}
