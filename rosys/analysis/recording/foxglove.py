"""Converters from RoSys data types to Foxglove JSON messages for MCAP.

Messages are plain JSON (human-readable) using Foxglove's well-known ``foxglove.*``
schemas where they exist, so Foxglove renders them natively (3D, Image, Map,
overlays); everything else is a small custom JSON schema that stays plottable.

These builders return :class:`Converter` objects and register themselves for
auto-dispatch; the wiring machinery (registry, topic sources, ``custom_message``)
lives in :mod:`.converters`.
"""
from __future__ import annotations

import base64
import math
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from ...geometry import Pose, Velocity
from ...hardware.bms_state import BmsState
from ...hardware.gnss.gnss import GnssMeasurement
from ...hardware.imu import ImuMeasurement
from ...vision import CalibratableCamera, Image
from .converters import Converter, add_event_topic, custom_message, register
from .mcap_recorder import NANOSECONDS_PER_SECOND, McapRecorder

if TYPE_CHECKING:
    from nicegui import Event


# --------------------------------------------------------------------------- #
# Shared JSON helpers and schema fragments
# --------------------------------------------------------------------------- #


def _foxglove_time(timestamp_ns: int) -> dict:
    return {'sec': timestamp_ns // NANOSECONDS_PER_SECOND, 'nsec': timestamp_ns % NANOSECONDS_PER_SECOND}


def _quaternion_from_yaw(yaw: float) -> dict:
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}


_TIME_SCHEMA = {'type': 'object', 'properties': {'sec': {'type': 'integer'}, 'nsec': {'type': 'integer'}}}
_VECTOR3_SCHEMA = {'type': 'object',
                   'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}}}
_QUATERNION_SCHEMA = {'type': 'object',
                      'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'},
                                     'z': {'type': 'number'}, 'w': {'type': 'number'}}}
_COLOR_SCHEMA = {'type': 'object', 'properties': {k: {'type': 'number'} for k in 'rgba'}}
_POINT2_SCHEMA = {'type': 'object', 'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}}}


# --------------------------------------------------------------------------- #
# Pose topics: PoseInFrame + FrameTransform
# --------------------------------------------------------------------------- #


def add_pose_topic(recorder: McapRecorder, topic: str, *,
                   event: Event,
                   child: str,
                   frame: str = 'map',
                   extract: Callable[[Any], Any] | None = None,
                   timestamp: Callable[[Any], float] | None = None) -> None:
    """Record a pose as both a ``foxglove.PoseInFrame`` and a ``foxglove.FrameTransform``.

    ``extract`` maps the event payload to a ``Pose`` (returning ``None`` to skip),
    so the same source can feed both. Give each pose source a distinct ``child``
    so the transforms form a valid tree under ``frame``.
    """
    add_event_topic(recorder, topic, event=event, timestamp=timestamp,
                    converter=pose_in_frame(frame=frame, extract=extract))
    add_event_topic(recorder, '/tf', event=event, timestamp=timestamp,
                    converter=frame_transform(parent=frame, child=child, extract=extract))


def pose_in_frame(*, frame: str = 'map', extract: Callable[[Any], Any] | None = None) -> Converter:
    """``Pose`` -> ``foxglove.PoseInFrame`` (yaw mapped to a z-axis quaternion)."""
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'frame_id': {'type': 'string'},
            'pose': {'type': 'object',
                     'properties': {'position': _VECTOR3_SCHEMA, 'orientation': _QUATERNION_SCHEMA}},
        },
    }

    def build(value: Any, timestamp_ns: int) -> dict | None:
        pose = extract(value) if extract is not None else value
        if pose is None:
            return None
        return {
            'timestamp': _foxglove_time(timestamp_ns), 'frame_id': frame,
            'pose': {'position': {'x': pose.x, 'y': pose.y, 'z': 0.0},
                     'orientation': _quaternion_from_yaw(pose.yaw)},
        }

    return custom_message('foxglove.PoseInFrame', schema, build)


def frame_transform(*, child: str, parent: str = 'map',
                    extract: Callable[[Any], Any] | None = None) -> Converter:
    """``Pose`` -> ``foxglove.FrameTransform`` ``parent`` -> ``child`` (2D, yaw quaternion)."""
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'parent_frame_id': {'type': 'string'},
            'child_frame_id': {'type': 'string'},
            'translation': _VECTOR3_SCHEMA, 'rotation': _QUATERNION_SCHEMA,
        },
    }

    def build(value: Any, timestamp_ns: int) -> dict | None:
        pose = extract(value) if extract is not None else value
        if pose is None:
            return None
        return {
            'timestamp': _foxglove_time(timestamp_ns),
            'parent_frame_id': parent, 'child_frame_id': child,
            'translation': {'x': pose.x, 'y': pose.y, 'z': 0.0},
            'rotation': _quaternion_from_yaw(pose.yaw),
        }

    return custom_message('foxglove.FrameTransform', schema, build)


def transform_3d(*, child: str, parent: str = 'base_link',
                 extract: Callable[[Any], Any] | None = None) -> Converter:
    """``Pose3d`` -> ``foxglove.FrameTransform`` (full 3D rotation, e.g. camera mount)."""
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'parent_frame_id': {'type': 'string'},
            'child_frame_id': {'type': 'string'},
            'translation': _VECTOR3_SCHEMA, 'rotation': _QUATERNION_SCHEMA,
        },
    }

    def build(value: Any, timestamp_ns: int) -> dict | None:
        pose = extract(value) if extract is not None else value
        if pose is None:
            return None
        w, x, y, z = pose.rotation.quaternion
        return {
            'timestamp': _foxglove_time(timestamp_ns),
            'parent_frame_id': parent, 'child_frame_id': child,
            'translation': {'x': pose.x, 'y': pose.y, 'z': pose.z},
            'rotation': {'x': x, 'y': y, 'z': z, 'w': w},
        }

    return custom_message('foxglove.FrameTransform', schema, build)


# --------------------------------------------------------------------------- #
# GNSS: LocationFix and status
# --------------------------------------------------------------------------- #

_COVARIANCE_TYPE_UNKNOWN = 0
_COVARIANCE_TYPE_DIAGONAL_KNOWN = 2  # foxglove.LocationFix.PositionCovarianceType.DIAGONAL_KNOWN


def location_fix(*, frame: str = 'gnss') -> Converter:
    """``GnssMeasurement`` -> ``foxglove.LocationFix`` (radians mapped to degrees).

    The horizontal position standard deviations, when known, are written as a
    diagonal 3x3 ``position_covariance`` in m² (row-major, East/North/Up). The
    receiver reports no vertical standard deviation, so the Up variance stays 0
    and the covariance type is ``DIAGONAL_KNOWN`` only. When no standard
    deviations are reported the covariance is left zeroed with type ``UNKNOWN``.
    """
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'frame_id': {'type': 'string'},
            'latitude': {'type': 'number'}, 'longitude': {'type': 'number'}, 'altitude': {'type': 'number'},
            'position_covariance': {'type': 'array', 'items': {'type': 'number'}},
            'position_covariance_type': {'type': 'integer'},
        },
    }

    def build(measurement: GnssMeasurement, timestamp_ns: int) -> dict:
        latitude_std_dev = measurement.latitude_std_dev
        longitude_std_dev = measurement.longitude_std_dev
        has_covariance = (math.isfinite(latitude_std_dev) and math.isfinite(longitude_std_dev)
                          and (latitude_std_dev > 0 or longitude_std_dev > 0))
        covariance = [0.0] * 9
        covariance_type = _COVARIANCE_TYPE_UNKNOWN
        if has_covariance:
            covariance[0] = longitude_std_dev ** 2  # East variance (m²)
            covariance[4] = latitude_std_dev ** 2  # North variance (m²)
            # Up variance stays 0: GnssMeasurement carries no altitude standard deviation.
            covariance_type = _COVARIANCE_TYPE_DIAGONAL_KNOWN
        return {
            'timestamp': _foxglove_time(timestamp_ns), 'frame_id': frame,
            'latitude': math.degrees(measurement.pose.lat),
            'longitude': math.degrees(measurement.pose.lon),
            'altitude': measurement.altitude,
            'position_covariance': covariance,
            'position_covariance_type': covariance_type,
        }

    return custom_message('foxglove.LocationFix', schema, build)


def describe_gnss_status(measurement: GnssMeasurement) -> dict:
    """Quality, satellites, hdop, heading and std devs of a GNSS fix as JSON-safe values.

    Shared by the ``/gnss/status`` recording topic (see :func:`gnss_status`). A
    single-antenna receiver reports no heading: ``heading_std_dev`` is non-finite
    and ``heading`` may be ``NaN``, neither of which is valid JSON — both are reported as
    ``None`` then, with ``heading_available`` set to ``False``.

    :param measurement: the GNSS measurement to describe.
    :return: a dict with quality (name), num_satellites, hdop, heading (radians or None),
        heading_std_dev (degrees or None), heading_available, and the position std devs
        in meters.
    """
    heading_available = math.isfinite(measurement.heading_std_dev)
    return {
        'quality': measurement.gps_quality.name,
        'num_satellites': measurement.num_satellites,
        'hdop': measurement.hdop,
        'heading': measurement.heading if math.isfinite(measurement.heading) else None,
        'heading_std_dev': measurement.heading_std_dev if heading_available else None,
        'heading_available': heading_available,
        'latitude_std_dev': measurement.latitude_std_dev,
        'longitude_std_dev': measurement.longitude_std_dev,
    }


_GNSS_STATUS_SCHEMA = {
    'type': 'object',
    'properties': {
        'quality': {'type': 'string'},
        'num_satellites': {'type': 'integer'},
        'hdop': {'type': 'number'},
        'heading': {'type': ['number', 'null']},  # null when the receiver reports no finite heading
        'heading_std_dev': {'type': ['number', 'null']},  # null when the fix carries no heading
        'heading_available': {'type': 'boolean'},
        'latitude_std_dev': {'type': 'number'},
        'longitude_std_dev': {'type': 'number'},
    },
}


def gnss_status() -> Converter:
    """``GnssMeasurement`` -> quality, satellites, hdop, heading and std devs (plottable).

    A non-finite ``heading_std_dev`` (single antenna, no heading) is written as ``null``
    with ``heading_available`` False, so the record stays valid JSON (``inf`` is not).
    """
    def build(measurement: GnssMeasurement, _timestamp_ns: int) -> dict:
        return describe_gnss_status(measurement)

    return custom_message('GnssStatus', _GNSS_STATUS_SCHEMA, build)


# --------------------------------------------------------------------------- #
# Battery -> sensor_msgs/msg/BatteryState
# --------------------------------------------------------------------------- #

# sensor_msgs/msg/BatteryState.power_supply_status enum (foxglove renders the ROS type natively)
_POWER_SUPPLY_STATUS_UNKNOWN = 0
_POWER_SUPPLY_STATUS_CHARGING = 1
_POWER_SUPPLY_STATUS_DISCHARGING = 2
_ROS_TIME_SCHEMA = {'type': 'object', 'properties': {'sec': {'type': 'integer'}, 'nanosec': {'type': 'integer'}}}
_NULLABLE_NUMBER = {'type': ['number', 'null']}  # ROS uses NaN for unmeasured; JSON can't, so null


def battery_state(*, frame: str = 'battery') -> Converter:
    """``BmsState`` -> ``sensor_msgs/msg/BatteryState`` (percentage rescaled 0..100 -> 0..1)."""
    schema = {
        'type': 'object',
        'properties': {
            'header': {'type': 'object',
                       'properties': {'stamp': _ROS_TIME_SCHEMA, 'frame_id': {'type': 'string'}}},
            'voltage': _NULLABLE_NUMBER, 'temperature': _NULLABLE_NUMBER, 'current': _NULLABLE_NUMBER,
            'charge': _NULLABLE_NUMBER, 'capacity': _NULLABLE_NUMBER, 'design_capacity': _NULLABLE_NUMBER,
            'percentage': _NULLABLE_NUMBER,
            'power_supply_status': {'type': 'integer'}, 'power_supply_health': {'type': 'integer'},
            'power_supply_technology': {'type': 'integer'}, 'present': {'type': 'boolean'},
            'cell_voltage': {'type': 'array', 'items': {'type': 'number'}},
            'cell_temperature': {'type': 'array', 'items': {'type': 'number'}},
            'location': {'type': 'string'}, 'serial_number': {'type': 'string'},
        },
    }

    def build(state: BmsState, timestamp_ns: int) -> dict:
        if state.is_charging is None:
            status = _POWER_SUPPLY_STATUS_UNKNOWN
        else:
            status = _POWER_SUPPLY_STATUS_CHARGING if state.is_charging else _POWER_SUPPLY_STATUS_DISCHARGING
        return {
            'header': {'stamp': {'sec': timestamp_ns // NANOSECONDS_PER_SECOND,
                                 'nanosec': timestamp_ns % NANOSECONDS_PER_SECOND},
                       'frame_id': frame},
            'voltage': state.voltage, 'temperature': state.temperature, 'current': state.current,
            'charge': None, 'capacity': None, 'design_capacity': None,  # not provided by the BMS
            'percentage': state.percentage / 100 if state.percentage is not None else None,
            'power_supply_status': status,
            'power_supply_health': 0,  # POWER_SUPPLY_HEALTH_UNKNOWN
            'power_supply_technology': 0,  # POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            'present': True,
            'cell_voltage': [], 'cell_temperature': [], 'location': '', 'serial_number': '',
        }

    return custom_message('sensor_msgs/msg/BatteryState', schema, build)


# --------------------------------------------------------------------------- #
# Camera: compressed image, calibration and detection overlays
# --------------------------------------------------------------------------- #


def compressed_image(*, frame: str = 'camera') -> Converter:
    """``Image`` -> ``foxglove.CompressedImage`` (JPEG, base64-encoded for JSON)."""
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'frame_id': {'type': 'string'},
            'data': {'type': 'string', 'contentEncoding': 'base64'}, 'format': {'type': 'string'},
        },
    }

    def build(image: Image, timestamp_ns: int) -> dict:
        return {
            'timestamp': _foxglove_time(timestamp_ns), 'frame_id': frame,
            'data': base64.b64encode(image.to_jpeg_bytes()).decode('ascii'), 'format': 'jpeg',
        }

    return custom_message('foxglove.CompressedImage', schema, build)


def camera_calibration(camera: CalibratableCamera, *, frame: str) -> Converter:
    """A ``CalibratableCamera`` -> ``foxglove.CameraCalibration`` (intrinsics K/D/P).

    Reads ``camera.calibration`` on each call; returns ``None`` while uncalibrated.
    """
    schema = {
        'type': 'object',
        'properties': {
            'timestamp': _TIME_SCHEMA, 'frame_id': {'type': 'string'},
            'width': {'type': 'integer'}, 'height': {'type': 'integer'},
            'distortion_model': {'type': 'string'},
            'D': {'type': 'array', 'items': {'type': 'number'}},
            'K': {'type': 'array', 'items': {'type': 'number'}},
            'R': {'type': 'array', 'items': {'type': 'number'}},
            'P': {'type': 'array', 'items': {'type': 'number'}},
        },
    }
    distortion_models = {'pinhole': 'plumb_bob', 'fisheye': 'equidistant'}

    def build(_value: Any, timestamp_ns: int) -> dict | None:
        calibration = camera.calibration
        if calibration is None:
            return None
        intrinsics = calibration.intrinsics
        matrix = [float(v) for row in intrinsics.matrix for v in row]
        fx, fy, cx, cy = matrix[0], matrix[4], matrix[2], matrix[5]
        return {
            'timestamp': _foxglove_time(timestamp_ns), 'frame_id': frame,
            'width': intrinsics.size.width, 'height': intrinsics.size.height,
            'distortion_model': distortion_models.get(intrinsics.model.value, ''),
            'D': [float(d) for d in intrinsics.distortion],
            'K': matrix,
            'R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'P': [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }

    return custom_message('foxglove.CameraCalibration', schema, build)


def image_annotations(*, camera_id: str | None = None, thickness: float = 2.0) -> Converter:
    """``Image`` with detections -> ``foxglove.ImageAnnotations`` overlay.

    Boxes become line-loop outlines, point detections circles, each with a
    ``category (confidence)`` label. With ``camera_id`` set, images from other
    cameras are skipped (for routing a shared detector stream per camera).
    """
    schema = {
        'type': 'object',
        'properties': {
            'circles': {'type': 'array', 'items': _CIRCLE_ANNOTATION_SCHEMA},
            'points': {'type': 'array', 'items': _POINTS_ANNOTATION_SCHEMA},
            'texts': {'type': 'array', 'items': _TEXT_ANNOTATION_SCHEMA},
        },
    }

    def build(image: Image, timestamp_ns: int) -> dict | None:
        if camera_id is not None and image.camera_id != camera_id:
            return None
        time = _foxglove_time(timestamp_ns)
        points: list[dict] = []
        circles: list[dict] = []
        texts: list[dict] = []
        for detections in image.detections_by_detector_id.values():
            for box in detections.boxes:
                points.append(_line_loop(time, [
                    (box.x, box.y), (box.x + box.width, box.y),
                    (box.x + box.width, box.y + box.height), (box.x, box.y + box.height),
                ], thickness))
                texts.append(_text(time, box.x, box.y, f'{box.category_name} ({box.confidence:.2f})'))
            for point in detections.points:
                circles.append(_circle(time, point.x, point.y, thickness))
                texts.append(_text(time, point.x, point.y, f'{point.category_name} ({point.confidence:.2f})'))
        return {'circles': circles, 'points': points, 'texts': texts}

    return custom_message('foxglove.ImageAnnotations', schema, build)


# --------------------------------------------------------------------------- #
# Plottable domain converters (no foxglove well-known schema)
# --------------------------------------------------------------------------- #


def velocity() -> Converter:
    """``Velocity`` -> ``{linear, angular}`` (plottable)."""
    schema = {'type': 'object',
              'properties': {'linear': {'type': 'number'}, 'angular': {'type': 'number'}}}
    return custom_message('Velocity', schema, lambda v, _ts: {'linear': v.linear, 'angular': v.angular})


def imu() -> Converter:
    """``ImuMeasurement`` -> roll/pitch/yaw, rates and gyro calibration (plottable)."""
    schema = {'type': 'object', 'properties': {k: {'type': 'number'} for k in (
        'roll', 'pitch', 'yaw', 'angular_velocity_roll', 'angular_velocity_pitch',
        'angular_velocity_yaw', 'gyro_calibration')}}

    def build(measurement: ImuMeasurement, _timestamp_ns: int) -> dict:
        rotation, angular = measurement.rotation, measurement.angular_velocity
        return {
            'roll': rotation.roll, 'pitch': rotation.pitch, 'yaw': rotation.yaw,
            'angular_velocity_roll': angular.roll, 'angular_velocity_pitch': angular.pitch,
            'angular_velocity_yaw': angular.yaw, 'gyro_calibration': measurement.gyro_calibration,
        }

    return custom_message('ImuMeasurement', schema, build)


# --------------------------------------------------------------------------- #
# Image-annotation sub-builders
# --------------------------------------------------------------------------- #

_CIRCLE_ANNOTATION_SCHEMA = {
    'type': 'object',
    'properties': {'timestamp': _TIME_SCHEMA, 'position': _POINT2_SCHEMA, 'diameter': {'type': 'number'},
                   'thickness': {'type': 'number'}, 'outline_color': _COLOR_SCHEMA, 'fill_color': _COLOR_SCHEMA},
}
_POINTS_ANNOTATION_SCHEMA = {
    'type': 'object',
    'properties': {'timestamp': _TIME_SCHEMA, 'type': {'type': 'integer'},
                   'points': {'type': 'array', 'items': _POINT2_SCHEMA},
                   'outline_color': _COLOR_SCHEMA, 'fill_color': _COLOR_SCHEMA, 'thickness': {'type': 'number'}},
}
_TEXT_ANNOTATION_SCHEMA = {
    'type': 'object',
    'properties': {'timestamp': _TIME_SCHEMA, 'position': _POINT2_SCHEMA, 'text': {'type': 'string'},
                   'font_size': {'type': 'number'}, 'text_color': _COLOR_SCHEMA, 'background_color': _COLOR_SCHEMA},
}
_RED = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
_TRANSPARENT = {'r': 0.0, 'g': 0.0, 'b': 0.0, 'a': 0.0}
_WHITE = {'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 1.0}
_LINE_LOOP = 2  # foxglove PointsAnnotationType.LINE_LOOP


def _line_loop(time: dict, corners: list[tuple[float, float]], thickness: float) -> dict:
    return {'timestamp': time, 'type': _LINE_LOOP, 'points': [{'x': x, 'y': y} for x, y in corners],
            'outline_color': _RED, 'fill_color': _TRANSPARENT, 'thickness': thickness}


def _circle(time: dict, x: float, y: float, thickness: float) -> dict:
    return {'timestamp': time, 'position': {'x': x, 'y': y}, 'diameter': thickness * 3, 'thickness': thickness,
            'outline_color': _RED, 'fill_color': _TRANSPARENT}


def _text(time: dict, x: float, y: float, text_value: str) -> dict:
    return {'timestamp': time, 'position': {'x': x, 'y': y}, 'text': text_value,
            'font_size': 12.0, 'text_color': _WHITE, 'background_color': _RED}


register(Pose, pose_in_frame)
register(Velocity, velocity)
register(GnssMeasurement, location_fix)
register(BmsState, battery_state)
register(ImuMeasurement, imu)
register(Image, compressed_image)
