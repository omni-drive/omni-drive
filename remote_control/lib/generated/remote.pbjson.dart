//
//  Generated code. Do not modify.
//  source: remote.proto
//
// @dart = 2.12

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_final_fields
// ignore_for_file: unnecessary_import, unnecessary_this, unused_import

import 'dart:convert' as $convert;
import 'dart:core' as $core;
import 'dart:typed_data' as $typed_data;

@$core.Deprecated('Use setSpeedRequestDescriptor instead')
const SetSpeedRequest$json = {
  '1': 'SetSpeedRequest',
  '2': [
    {'1': 'x', '3': 1, '4': 1, '5': 2, '10': 'x'},
    {'1': 'y', '3': 2, '4': 1, '5': 2, '10': 'y'},
    {'1': 'omega', '3': 3, '4': 1, '5': 2, '10': 'omega'},
    {'1': 'reset_pose', '3': 4, '4': 1, '5': 8, '10': 'resetPose'},
  ],
};

/// Descriptor for `SetSpeedRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List setSpeedRequestDescriptor = $convert.base64Decode(
    'Cg9TZXRTcGVlZFJlcXVlc3QSDAoBeBgBIAEoAlIBeBIMCgF5GAIgASgCUgF5EhQKBW9tZWdhGA'
    'MgASgCUgVvbWVnYRIdCgpyZXNldF9wb3NlGAQgASgIUglyZXNldFBvc2U=');

@$core.Deprecated('Use distanceSensorsDescriptor instead')
const DistanceSensors$json = {
  '1': 'DistanceSensors',
  '2': [
    {'1': 'left_m', '3': 1, '4': 1, '5': 2, '10': 'leftM'},
    {'1': 'front_m', '3': 2, '4': 1, '5': 2, '10': 'frontM'},
    {'1': 'right_m', '3': 3, '4': 1, '5': 2, '10': 'rightM'},
  ],
};

/// Descriptor for `DistanceSensors`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List distanceSensorsDescriptor = $convert.base64Decode(
    'Cg9EaXN0YW5jZVNlbnNvcnMSFQoGbGVmdF9tGAEgASgCUgVsZWZ0TRIXCgdmcm9udF9tGAIgAS'
    'gCUgZmcm9udE0SFwoHcmlnaHRfbRgDIAEoAlIGcmlnaHRN');

@$core.Deprecated('Use robotTelemetryDescriptor instead')
const RobotTelemetry$json = {
  '1': 'RobotTelemetry',
  '2': [
    {'1': 'battery_voltage', '3': 1, '4': 1, '5': 2, '10': 'batteryVoltage'},
    {'1': 'battery_percentage', '3': 2, '4': 1, '5': 2, '10': 'batteryPercentage'},
    {'1': 'is_battery_critical', '3': 3, '4': 1, '5': 8, '10': 'isBatteryCritical'},
    {'1': 'x', '3': 4, '4': 1, '5': 2, '10': 'x'},
    {'1': 'y', '3': 5, '4': 1, '5': 2, '10': 'y'},
    {'1': 'yaw_rad', '3': 6, '4': 1, '5': 2, '10': 'yawRad'},
    {'1': 'yaw_deg', '3': 7, '4': 1, '5': 2, '10': 'yawDeg'},
    {'1': 'tof_sensors', '3': 8, '4': 1, '5': 11, '6': '.remote.DistanceSensors', '10': 'tofSensors'},
    {'1': 'ultrasonic_sensors', '3': 9, '4': 1, '5': 11, '6': '.remote.DistanceSensors', '10': 'ultrasonicSensors'},
  ],
};

/// Descriptor for `RobotTelemetry`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List robotTelemetryDescriptor = $convert.base64Decode(
    'Cg5Sb2JvdFRlbGVtZXRyeRInCg9iYXR0ZXJ5X3ZvbHRhZ2UYASABKAJSDmJhdHRlcnlWb2x0YW'
    'dlEi0KEmJhdHRlcnlfcGVyY2VudGFnZRgCIAEoAlIRYmF0dGVyeVBlcmNlbnRhZ2USLgoTaXNf'
    'YmF0dGVyeV9jcml0aWNhbBgDIAEoCFIRaXNCYXR0ZXJ5Q3JpdGljYWwSDAoBeBgEIAEoAlIBeB'
    'IMCgF5GAUgASgCUgF5EhcKB3lhd19yYWQYBiABKAJSBnlhd1JhZBIXCgd5YXdfZGVnGAcgASgC'
    'UgZ5YXdEZWcSOAoLdG9mX3NlbnNvcnMYCCABKAsyFy5yZW1vdGUuRGlzdGFuY2VTZW5zb3JzUg'
    'p0b2ZTZW5zb3JzEkYKEnVsdHJhc29uaWNfc2Vuc29ycxgJIAEoCzIXLnJlbW90ZS5EaXN0YW5j'
    'ZVNlbnNvcnNSEXVsdHJhc29uaWNTZW5zb3Jz');

