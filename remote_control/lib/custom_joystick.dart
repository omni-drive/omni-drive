import 'package:flutter/material.dart';

class CustomJoystick extends StatefulWidget {
  final double size;
  final void Function(double x, double y) onChanged;
  final VoidCallback onEnd;

  const CustomJoystick({
    super.key,
    required this.size,
    required this.onChanged,
    required this.onEnd,
  });

  @override
  State<CustomJoystick> createState() => _CustomJoystickState();
}

class _CustomJoystickState extends State<CustomJoystick>
    with SingleTickerProviderStateMixin {
  Offset _stickOffset = Offset.zero;
  bool _isDragging = false;
  Offset _grabOffset = Offset.zero;
  late final AnimationController _resetController;
  Animation<Offset>? _resetAnimation;
  static const Duration _returnDuration = Duration(milliseconds: 200);

  double get _baseRadius => widget.size / 2;
  double get _maxRadius => _baseRadius * 0.75;
  double get _stickRadius => _baseRadius * 0.25;
  double get _hitRadius => _stickRadius * 1.35;

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
    setState(() => _stickOffset = Offset.zero);
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
    setState(() => _stickOffset = _resetAnimation!.value);
    _notifyPosition();
  }

  void _notifyPosition() {
    final x = (_stickOffset.dx / _maxRadius).clamp(-1.0, 1.0);
    final y = (_stickOffset.dy / _maxRadius).clamp(-1.0, 1.0);
    widget.onChanged(x, -y);
  }

  void _handlePanStart(DragStartDetails details) {
    _resetController.stop();
    final center = Offset(widget.size / 2, widget.size / 2);
    final touchPosition = details.localPosition;
    final stickCenter = center + _stickOffset;
    final distanceToStick = (touchPosition - stickCenter).distance;
    if (distanceToStick > _hitRadius) {
      _isDragging = false;
      return;
    }
    _isDragging = true;
    _grabOffset = touchPosition - stickCenter;
  }

  void _handlePanUpdate(DragUpdateDetails details) {
    if (!_isDragging) return;
    final center = Offset(widget.size / 2, widget.size / 2);
    final desiredStickCenter = details.localPosition - _grabOffset;
    Offset delta = desiredStickCenter - center;
    if (delta.distance > _maxRadius) {
      delta = Offset.fromDirection(delta.direction, _maxRadius);
    }
    setState(() => _stickOffset = delta);
    _notifyPosition();
  }

  void _handlePanEnd() {
    if (!_isDragging) return;
    _isDragging = false;
    _grabOffset = Offset.zero;
    _startReturnAnimation();
  }

  void _startReturnAnimation() {
    _resetController.stop();
    _resetAnimation = Tween<Offset>(
      begin: _stickOffset,
      end: Offset.zero,
    ).animate(CurvedAnimation(
      parent: _resetController,
      curve: Curves.easeOutCubic,
    ));
    _resetController.forward(from: 0.0);
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: ClipOval(
        child: GestureDetector(
          behavior: HitTestBehavior.opaque,
          onPanStart: _handlePanStart,
          onPanUpdate: _handlePanUpdate,
          onPanEnd: (_) => _handlePanEnd(),
          onPanCancel: _handlePanEnd,
          child: CustomPaint(
            size: Size(widget.size, widget.size),
            painter: _JoystickPainter(stickOffset: _stickOffset),
          ),
        ),
      ),
    );
  }
}

class _JoystickPainter extends CustomPainter {
  final Offset stickOffset;

  const _JoystickPainter({required this.stickOffset});

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final baseRadius = size.width / 2;
    final basePaint = Paint()
      ..color = Colors.white.withOpacity(0.08)
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, baseRadius, basePaint);
    final borderPaint = Paint()
      ..color = Colors.white24
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawCircle(center, baseRadius, borderPaint);
    final centerPointPaint = Paint()
      ..color = Colors.white38
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, 4, centerPointPaint);
    final stickCenter = center + stickOffset;
    final stickPaint = Paint()
      ..color = Colors.blueAccent
      ..style = PaintingStyle.fill;
    canvas.drawCircle(stickCenter, baseRadius * 0.25, stickPaint);
    final stickBorderPaint = Paint()
      ..color = Colors.white.withOpacity(0.8)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawCircle(stickCenter, baseRadius * 0.25, stickBorderPaint);
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter oldDelegate) {
    return oldDelegate.stickOffset != stickOffset;
  }
}
