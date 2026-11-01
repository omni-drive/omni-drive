import 'dart:math' as math;

import 'package:flutter/material.dart';

class ArcRotationSlider extends StatefulWidget {
  final double size;
  final ValueChanged<double> onChanged;
  final VoidCallback onEnd;

  const ArcRotationSlider({
    super.key,
    required this.size,
    required this.onChanged,
    required this.onEnd,
  });

  @override
  State<ArcRotationSlider> createState() => _ArcRotationSliderState();
}

class _ArcRotationSliderState extends State<ArcRotationSlider>
    with SingleTickerProviderStateMixin {
  double _value = 0.0;
  bool _isDragging = false;
  Offset _grabOffset = Offset.zero;
  late final AnimationController _resetController;
  Animation<double>? _resetAnimation;
  static const double _maxAngleRad = math.pi / 3.0;
  static const Duration _returnDuration = Duration(milliseconds: 200);

  double get _radius => (widget.size / 2) - 16;
  double get _hitRadius => 20.0;
  double get _centerAngle => -math.pi / 2;
  Offset get _center => Offset(widget.size / 2, widget.size / 2);

  Offset get _thumbPosition {
    final angle = _centerAngle + (_value * _maxAngleRad);
    return Offset(
      _center.dx + _radius * math.cos(angle),
      _center.dy + _radius * math.sin(angle),
    );
  }

  @override
  void initState() {
    super.initState();
    _resetController = AnimationController(
      vsync: this,
      duration: _returnDuration,
    );
    _resetController.addListener(_onResetAnimation);
    _resetController.addStatusListener(_onResetStatus);
  }

  void _onResetStatus(AnimationStatus status) {
    if (status != AnimationStatus.completed) return;
    if (_isDragging) return;
    if (!mounted) return;

    setState(() => _value = 0.0);
    widget.onEnd();
  }

  @override
  void dispose() {
    _resetController.dispose();
    super.dispose();
  }

  void _onResetAnimation() {
    if (_resetAnimation == null) return;
    if (!mounted) return;
    setState(() => _value = _resetAnimation!.value);
    widget.onChanged(_value);
  }

  double _normalizeAngle(double angle) {
    while (angle > math.pi) {
      angle -= 2 * math.pi;
    }

    while (angle < -math.pi) {
      angle += 2 * math.pi;
    }

    return angle;
  }

  double _valueFromPosition(Offset position) {
    final vector = position - _center;
    if (vector.distance < 0.001) return 0.0;
    final angle = math.atan2(vector.dy, vector.dx);
    final diff = _normalizeAngle(angle - _centerAngle);
    return (diff / _maxAngleRad).clamp(-1.0, 1.0);
  }

  void _handlePanStart(DragStartDetails details) {
    _resetController.stop();
    final touchPosition = details.localPosition;
    final thumbPosition = _thumbPosition;
    final distance = (touchPosition - thumbPosition).distance;
    if (distance > _hitRadius) {
      _isDragging = false;
      return;
    }
    _isDragging = true;
    _grabOffset = touchPosition - thumbPosition;
  }

  void _handlePanUpdate(DragUpdateDetails details) {
    if (!_isDragging) return;
    final correctedPosition = details.localPosition - _grabOffset;
    final newValue = _valueFromPosition(correctedPosition);
    setState(() => _value = newValue);
    widget.onChanged(_value);
  }

  void _handlePanEnd() {
    if (!_isDragging) return;
    _isDragging = false;
    _grabOffset = Offset.zero;
    _startReturnAnimation();
  }

  void _startReturnAnimation() {
    _resetController.stop();
    _resetAnimation = Tween<double>(
      begin: _value,
      end: 0.0,
    ).animate(CurvedAnimation(
      parent: _resetController,
      curve: Curves.easeOutCubic,
    ));
    _resetController.forward(from: 0.0);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onPanStart: _handlePanStart,
      onPanUpdate: _handlePanUpdate,
      onPanEnd: (_) => _handlePanEnd(),
      onPanCancel: _handlePanEnd,
      child: CustomPaint(
        size: Size(widget.size, widget.size),
        painter: _ArcSliderPainter(
          value: _value,
          maxAngle: _maxAngleRad,
        ),
      ),
    );
  }
}

class _ArcSliderPainter extends CustomPainter {
  final double value;
  final double maxAngle;

  const _ArcSliderPainter({
    required this.value,
    required this.maxAngle,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = (size.width / 2) - 16;
    final trackPaint = Paint()
      ..color = Colors.white24
      ..style = PaintingStyle.stroke
      ..strokeWidth = 6.0
      ..strokeCap = StrokeCap.round;
    final startAngle = -math.pi / 2 - maxAngle;
    final sweepAngle = maxAngle * 2;
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: radius),
      startAngle,
      sweepAngle,
      false,
      trackPaint,
    );
    final centerTickPaint = Paint()
      ..color = Colors.white54
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    final pTop = Offset(center.dx, center.dy - radius - 8);
    final pBottom = Offset(center.dx, center.dy - radius + 8);
    canvas.drawLine(pTop, pBottom, centerTickPaint);
    final thumbAngle = -math.pi / 2 + (value * maxAngle);
    final thumbPosition = Offset(
      center.dx + radius * math.cos(thumbAngle),
      center.dy + radius * math.sin(thumbAngle),
    );
    final thumbPaint = Paint()
      ..color = value.abs() < 0.0001 ? Colors.white70 : Colors.cyanAccent
      ..style = PaintingStyle.fill;
    canvas.drawCircle(thumbPosition, 12, thumbPaint);
    final thumbBorderPaint = Paint()
      ..color = Colors.white.withOpacity(0.8)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawCircle(thumbPosition, 12, thumbBorderPaint);
  }

  @override
  bool shouldRepaint(covariant _ArcSliderPainter oldDelegate) {
    return oldDelegate.value != value || oldDelegate.maxAngle != maxAngle;
  }
}
