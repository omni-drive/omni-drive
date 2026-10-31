//
//  Generated code. Do not modify.
//  source: remote.proto
//
// @dart = 2.12

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_final_fields
// ignore_for_file: unnecessary_import, unnecessary_this, unused_import

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

class SetSpeedRequest extends $pb.GeneratedMessage {
  factory SetSpeedRequest({
    $core.double? x,
    $core.double? y,
    $core.double? omega,
    $core.bool? resetPose,
  }) {
    final $result = create();
    if (x != null) {
      $result.x = x;
    }
    if (y != null) {
      $result.y = y;
    }
    if (omega != null) {
      $result.omega = omega;
    }
    if (resetPose != null) {
      $result.resetPose = resetPose;
    }
    return $result;
  }
  SetSpeedRequest._() : super();
  factory SetSpeedRequest.fromBuffer($core.List<$core.int> i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromBuffer(i, r);
  factory SetSpeedRequest.fromJson($core.String i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'SetSpeedRequest', package: const $pb.PackageName(_omitMessageNames ? '' : 'remote'), createEmptyInstance: create)
    ..a<$core.double>(1, _omitFieldNames ? '' : 'x', $pb.PbFieldType.OF)
    ..a<$core.double>(2, _omitFieldNames ? '' : 'y', $pb.PbFieldType.OF)
    ..a<$core.double>(3, _omitFieldNames ? '' : 'omega', $pb.PbFieldType.OF)
    ..aOB(4, _omitFieldNames ? '' : 'resetPose')
    ..hasRequiredFields = false
  ;

  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
  'Will be removed in next major version')
  SetSpeedRequest clone() => SetSpeedRequest()..mergeFromMessage(this);
  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
  'Will be removed in next major version')
  SetSpeedRequest copyWith(void Function(SetSpeedRequest) updates) => super.copyWith((message) => updates(message as SetSpeedRequest)) as SetSpeedRequest;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static SetSpeedRequest create() => SetSpeedRequest._();
  SetSpeedRequest createEmptyInstance() => create();
  static $pb.PbList<SetSpeedRequest> createRepeated() => $pb.PbList<SetSpeedRequest>();
  @$core.pragma('dart2js:noInline')
  static SetSpeedRequest getDefault() => _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<SetSpeedRequest>(create);
  static SetSpeedRequest? _defaultInstance;

  @$pb.TagNumber(1)
  $core.double get x => $_getN(0);
  @$pb.TagNumber(1)
  set x($core.double v) { $_setFloat(0, v); }
  @$pb.TagNumber(1)
  $core.bool hasX() => $_has(0);
  @$pb.TagNumber(1)
  void clearX() => clearField(1);

  @$pb.TagNumber(2)
  $core.double get y => $_getN(1);
  @$pb.TagNumber(2)
  set y($core.double v) { $_setFloat(1, v); }
  @$pb.TagNumber(2)
  $core.bool hasY() => $_has(1);
  @$pb.TagNumber(2)
  void clearY() => clearField(2);

  @$pb.TagNumber(3)
  $core.double get omega => $_getN(2);
  @$pb.TagNumber(3)
  set omega($core.double v) { $_setFloat(2, v); }
  @$pb.TagNumber(3)
  $core.bool hasOmega() => $_has(2);
  @$pb.TagNumber(3)
  void clearOmega() => clearField(3);

  @$pb.TagNumber(4)
  $core.bool get resetPose => $_getBF(3);
  @$pb.TagNumber(4)
  set resetPose($core.bool v) { $_setBool(3, v); }
  @$pb.TagNumber(4)
  $core.bool hasResetPose() => $_has(3);
  @$pb.TagNumber(4)
  void clearResetPose() => clearField(4);
}

class DistanceSensors extends $pb.GeneratedMessage {
  factory DistanceSensors({
    $core.double? leftM,
    $core.double? frontM,
    $core.double? rightM,
  }) {
    final $result = create();
    if (leftM != null) {
      $result.leftM = leftM;
    }
    if (frontM != null) {
      $result.frontM = frontM;
    }
    if (rightM != null) {
      $result.rightM = rightM;
    }
    return $result;
  }
  DistanceSensors._() : super();
  factory DistanceSensors.fromBuffer($core.List<$core.int> i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromBuffer(i, r);
  factory DistanceSensors.fromJson($core.String i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'DistanceSensors', package: const $pb.PackageName(_omitMessageNames ? '' : 'remote'), createEmptyInstance: create)
    ..a<$core.double>(1, _omitFieldNames ? '' : 'leftM', $pb.PbFieldType.OF)
    ..a<$core.double>(2, _omitFieldNames ? '' : 'frontM', $pb.PbFieldType.OF)
    ..a<$core.double>(3, _omitFieldNames ? '' : 'rightM', $pb.PbFieldType.OF)
    ..hasRequiredFields = false
  ;

  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
  'Will be removed in next major version')
  DistanceSensors clone() => DistanceSensors()..mergeFromMessage(this);
  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
  'Will be removed in next major version')
  DistanceSensors copyWith(void Function(DistanceSensors) updates) => super.copyWith((message) => updates(message as DistanceSensors)) as DistanceSensors;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static DistanceSensors create() => DistanceSensors._();
  DistanceSensors createEmptyInstance() => create();
  static $pb.PbList<DistanceSensors> createRepeated() => $pb.PbList<DistanceSensors>();
  @$core.pragma('dart2js:noInline')
  static DistanceSensors getDefault() => _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<DistanceSensors>(create);
  static DistanceSensors? _defaultInstance;

  @$pb.TagNumber(1)
  $core.double get leftM => $_getN(0);
  @$pb.TagNumber(1)
  set leftM($core.double v) { $_setFloat(0, v); }
  @$pb.TagNumber(1)
  $core.bool hasLeftM() => $_has(0);
  @$pb.TagNumber(1)
  void clearLeftM() => clearField(1);

  @$pb.TagNumber(2)
  $core.double get frontM => $_getN(1);
  @$pb.TagNumber(2)
  set frontM($core.double v) { $_setFloat(1, v); }
  @$pb.TagNumber(2)
  $core.bool hasFrontM() => $_has(1);
  @$pb.TagNumber(2)
  void clearFrontM() => clearField(2);

  @$pb.TagNumber(3)
  $core.double get rightM => $_getN(2);
  @$pb.TagNumber(3)
  set rightM($core.double v) { $_setFloat(2, v); }
  @$pb.TagNumber(3)
  $core.bool hasRightM() => $_has(2);
  @$pb.TagNumber(3)
  void clearRightM() => clearField(3);
}

class RobotTelemetry extends $pb.GeneratedMessage {
  factory RobotTelemetry({
    $core.double? batteryVoltage,
    $core.double? batteryPercentage,
    $core.bool? isBatteryCritical,
    $core.double? x,
    $core.double? y,
    $core.double? yawRad,
    $core.double? yawDeg,
    DistanceSensors? tofSensors,
    DistanceSensors? ultrasonicSensors,
  }) {
    final $result = create();
    if (batteryVoltage != null) {
      $result.batteryVoltage = batteryVoltage;
    }
    if (batteryPercentage != null) {
      $result.batteryPercentage = batteryPercentage;
    }
    if (isBatteryCritical != null) {
      $result.isBatteryCritical = isBatteryCritical;
    }
    if (x != null) {
      $result.x = x;
    }
    if (y != null) {
      $result.y = y;
    }
    if (yawRad != null) {
      $result.yawRad = yawRad;
    }
    if (yawDeg != null) {
      $result.yawDeg = yawDeg;
    }
    if (tofSensors != null) {
      $result.tofSensors = tofSensors;
    }
    if (ultrasonicSensors != null) {
      $result.ultrasonicSensors = ultrasonicSensors;
    }
    return $result;
  }
  RobotTelemetry._() : super();
  factory RobotTelemetry.fromBuffer($core.List<$core.int> i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromBuffer(i, r);
  factory RobotTelemetry.fromJson($core.String i, [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) => create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'RobotTelemetry', package: const $pb.PackageName(_omitMessageNames ? '' : 'remote'), createEmptyInstance: create)
    ..a<$core.double>(1, _omitFieldNames ? '' : 'batteryVoltage', $pb.PbFieldType.OF)
    ..a<$core.double>(2, _omitFieldNames ? '' : 'batteryPercentage', $pb.PbFieldType.OF)
    ..aOB(3, _omitFieldNames ? '' : 'isBatteryCritical')
    ..a<$core.double>(4, _omitFieldNames ? '' : 'x', $pb.PbFieldType.OF)
    ..a<$core.double>(5, _omitFieldNames ? '' : 'y', $pb.PbFieldType.OF)
    ..a<$core.double>(6, _omitFieldNames ? '' : 'yawRad', $pb.PbFieldType.OF)
    ..a<$core.double>(7, _omitFieldNames ? '' : 'yawDeg', $pb.PbFieldType.OF)
    ..aOM<DistanceSensors>(8, _omitFieldNames ? '' : 'tofSensors', subBuilder: DistanceSensors.create)
    ..aOM<DistanceSensors>(9, _omitFieldNames ? '' : 'ultrasonicSensors', subBuilder: DistanceSensors.create)
    ..hasRequiredFields = false
  ;

  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
  'Will be removed in next major version')
  RobotTelemetry clone() => RobotTelemetry()..mergeFromMessage(this);
  @$core.Deprecated(
  'Using this can add significant overhead to your binary. '
  'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
  'Will be removed in next major version')
  RobotTelemetry copyWith(void Function(RobotTelemetry) updates) => super.copyWith((message) => updates(message as RobotTelemetry)) as RobotTelemetry;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static RobotTelemetry create() => RobotTelemetry._();
  RobotTelemetry createEmptyInstance() => create();
  static $pb.PbList<RobotTelemetry> createRepeated() => $pb.PbList<RobotTelemetry>();
  @$core.pragma('dart2js:noInline')
  static RobotTelemetry getDefault() => _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<RobotTelemetry>(create);
  static RobotTelemetry? _defaultInstance;

  @$pb.TagNumber(1)
  $core.double get batteryVoltage => $_getN(0);
  @$pb.TagNumber(1)
  set batteryVoltage($core.double v) { $_setFloat(0, v); }
  @$pb.TagNumber(1)
  $core.bool hasBatteryVoltage() => $_has(0);
  @$pb.TagNumber(1)
  void clearBatteryVoltage() => clearField(1);

  @$pb.TagNumber(2)
  $core.double get batteryPercentage => $_getN(1);
  @$pb.TagNumber(2)
  set batteryPercentage($core.double v) { $_setFloat(1, v); }
  @$pb.TagNumber(2)
  $core.bool hasBatteryPercentage() => $_has(1);
  @$pb.TagNumber(2)
  void clearBatteryPercentage() => clearField(2);

  @$pb.TagNumber(3)
  $core.bool get isBatteryCritical => $_getBF(2);
  @$pb.TagNumber(3)
  set isBatteryCritical($core.bool v) { $_setBool(2, v); }
  @$pb.TagNumber(3)
  $core.bool hasIsBatteryCritical() => $_has(2);
  @$pb.TagNumber(3)
  void clearIsBatteryCritical() => clearField(3);

  @$pb.TagNumber(4)
  $core.double get x => $_getN(3);
  @$pb.TagNumber(4)
  set x($core.double v) { $_setFloat(3, v); }
  @$pb.TagNumber(4)
  $core.bool hasX() => $_has(3);
  @$pb.TagNumber(4)
  void clearX() => clearField(4);

  @$pb.TagNumber(5)
  $core.double get y => $_getN(4);
  @$pb.TagNumber(5)
  set y($core.double v) { $_setFloat(4, v); }
  @$pb.TagNumber(5)
  $core.bool hasY() => $_has(4);
  @$pb.TagNumber(5)
  void clearY() => clearField(5);

  @$pb.TagNumber(6)
  $core.double get yawRad => $_getN(5);
  @$pb.TagNumber(6)
  set yawRad($core.double v) { $_setFloat(5, v); }
  @$pb.TagNumber(6)
  $core.bool hasYawRad() => $_has(5);
  @$pb.TagNumber(6)
  void clearYawRad() => clearField(6);

  @$pb.TagNumber(7)
  $core.double get yawDeg => $_getN(6);
  @$pb.TagNumber(7)
  set yawDeg($core.double v) { $_setFloat(6, v); }
  @$pb.TagNumber(7)
  $core.bool hasYawDeg() => $_has(6);
  @$pb.TagNumber(7)
  void clearYawDeg() => clearField(7);

  @$pb.TagNumber(8)
  DistanceSensors get tofSensors => $_getN(7);
  @$pb.TagNumber(8)
  set tofSensors(DistanceSensors v) { setField(8, v); }
  @$pb.TagNumber(8)
  $core.bool hasTofSensors() => $_has(7);
  @$pb.TagNumber(8)
  void clearTofSensors() => clearField(8);
  @$pb.TagNumber(8)
  DistanceSensors ensureTofSensors() => $_ensure(7);

  @$pb.TagNumber(9)
  DistanceSensors get ultrasonicSensors => $_getN(8);
  @$pb.TagNumber(9)
  set ultrasonicSensors(DistanceSensors v) { setField(9, v); }
  @$pb.TagNumber(9)
  $core.bool hasUltrasonicSensors() => $_has(8);
  @$pb.TagNumber(9)
  void clearUltrasonicSensors() => clearField(9);
  @$pb.TagNumber(9)
  DistanceSensors ensureUltrasonicSensors() => $_ensure(8);
}


const _omitFieldNames = $core.bool.fromEnvironment('protobuf.omit_field_names');
const _omitMessageNames = $core.bool.fromEnvironment('protobuf.omit_message_names');
