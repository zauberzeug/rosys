import base64
import json
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from mcap.reader import make_reader
from nicegui import Client, Event
from nicegui.page import page

from rosys.analysis.recording import (
    Converter,
    McapRecorder,
    TopicSchema,
    add_event_topic,
    add_pose_topic,
    add_timer_topic,
    battery_state,
    camera_calibration,
    converter_for,
    converters,
    custom_message,
    frame_transform,
    gnss_status,
    image_annotations,
    location_fix,
    pose_in_frame,
    scalar,
    transform_3d,
)
from rosys.geometry import GeoPose, Pose, Pose3d, Rotation, Velocity
from rosys.hardware.bms_state import BmsState
from rosys.hardware.gnss import GnssMeasurement
from rosys.hardware.gnss.nmea import GpsQuality
from rosys.hardware.imu import ImuMeasurement
from rosys.vision import BoxDetection, Detections, Image, PointDetection

NS = 1_000_000_000


class FakeEvent:
    """Minimal stand-in for nicegui's Event (same subscribe/unsubscribe/emit surface)."""

    def __init__(self) -> None:
        self.callbacks: list = []

    def subscribe(self, callback, *, unsubscribe_on_delete: bool | None = None) -> None:
        # unsubscribe_on_delete mirrors nicegui's Event.subscribe; the fake has no client
        # binding, so it is accepted and ignored.
        self.callbacks.append(callback)

    def unsubscribe(self, callback) -> None:
        self.callbacks[:] = [c for c in self.callbacks if c != callback]

    def emit(self, *args) -> None:
        for callback in list(self.callbacks):
            callback(*args)


@pytest.fixture
def mcap_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield Path(tmp)


def _read(path: Path) -> list[tuple[str, dict]]:
    with open(path, 'rb') as f:
        reader = make_reader(f)
        return [(schema.name, json.loads(message.data)) for schema, _, message in reader.iter_messages()]


def _read_raw(path: Path) -> list[bytes]:
    """Return the raw (undecoded) message payloads, to assert on the exact JSON bytes.

    :param path: the MCAP file to read.
    :return: the raw ``data`` bytes of each recorded message, in file order.
    """
    with open(path, 'rb') as f:
        return [message.data for _, _, message in make_reader(f).iter_messages()]


def _only_file(mcap_dir: Path) -> Path:
    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) == 1
    return files[0]


# --------------------------------------------------------------------------- #
# Auto-dispatch to foxglove schemas
# --------------------------------------------------------------------------- #


def test_event_topic_auto_picks_pose_in_frame(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/pose', event=event)
    recorder.start()
    event.emit(Pose(x=1.0, y=2.0, yaw=math.pi / 2, time=3.0))
    recorder.stop()

    (schema_name, message), = _read(_only_file(mcap_dir))
    assert schema_name == 'foxglove.PoseInFrame'
    assert message['frame_id'] == 'map'
    assert message['timestamp'] == {'sec': 3, 'nsec': 0}
    assert message['pose']['position'] == {'x': 1.0, 'y': 2.0, 'z': 0.0}
    assert message['pose']['orientation']['z'] == pytest.approx(math.sin(math.pi / 4))


def test_event_topic_auto_picks_velocity(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/wheels', event=event)
    recorder.start()
    event.emit(Velocity(linear=0.5, angular=0.1, time=1.0))
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'Velocity'
    assert message == {'linear': 0.5, 'angular': 0.1}


def test_override_frame_forwarded_to_converter(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/pose', event=event, frame='odom')
    recorder.start()
    event.emit(Pose(x=0.0, y=0.0, yaw=0.0, time=1.0))
    recorder.stop()

    _, message = _read(_only_file(mcap_dir))[0]
    assert message['frame_id'] == 'odom'


def test_explicit_converter_frame_transform(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/tf', event=event, converter=frame_transform(parent='map', child='base_link'))
    recorder.start()
    event.emit(Pose(x=1.0, y=0.0, yaw=0.0, time=1.0))
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'foxglove.FrameTransform'
    assert message['parent_frame_id'] == 'map'
    assert message['child_frame_id'] == 'base_link'
    assert message['translation'] == {'x': 1.0, 'y': 0.0, 'z': 0.0}


def test_add_pose_topic_emits_pose_and_tf(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_pose_topic(recorder, '/odometry/pose', event=event, child='odometry')
    recorder.start()
    event.emit(Pose(x=1.0, y=2.0, yaw=0.0, time=1.0))
    recorder.stop()

    by_schema = dict(_read(_only_file(mcap_dir)))
    assert set(by_schema) == {'foxglove.PoseInFrame', 'foxglove.FrameTransform'}
    assert by_schema['foxglove.PoseInFrame']['pose']['position']['x'] == 1.0
    assert by_schema['foxglove.FrameTransform']['child_frame_id'] == 'odometry'


def test_add_pose_topic_extract_skips_none(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_pose_topic(recorder, '/gnss/local', event=event, child='gnss', extract=lambda _m: None)
    recorder.start()
    event.emit(object())
    recorder.stop()
    assert recorder._message_count == 0


def test_unpack_records_each_element(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/wheels', event=event, unpack=True)
    recorder.start()
    event.emit([Velocity(linear=0.1, angular=0.0, time=1.0),
                Velocity(linear=0.2, angular=0.0, time=2.0)])
    recorder.stop()

    assert [m['linear'] for _, m in _read(_only_file(mcap_dir))] == [0.1, 0.2]


def test_converter_for_unknown_type_raises() -> None:
    with pytest.raises(TypeError, match='no converter registered'):
        converter_for(object())


def test_real_nicegui_event_delivers_payload(mcap_dir: Path) -> None:
    # Regression: nicegui only passes the emitted payload if the handler has a
    # required positional parameter, so a `*args`-only handler received None.

    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event: Event = Event()
    add_event_topic(recorder, '/pose', event=event)
    recorder.start()
    event.emit(Pose(x=7.0, y=0.0, yaw=0.0, time=1.0))
    recorder.stop()

    _, message = _read(_only_file(mcap_dir))[0]
    assert message['pose']['position']['x'] == 7.0


def test_argless_event_records(mcap_dir: Path) -> None:

    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event: Event = Event()
    converter = custom_message('Marker', {'type': 'object', 'properties': {'event': {'type': 'string'}}},
                               lambda _payload, _ts: {'event': 'done'})
    add_event_topic(recorder, '/marker', event=event, argless=True, converter=converter)
    recorder.start()
    event.emit()  # no payload
    recorder.stop()

    _, message = _read(_only_file(mcap_dir))[0]
    assert message == {'event': 'done'}


def test_scalar_auto_dispatch(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    add_timer_topic(recorder, '/temperature', interval=0.1, read=lambda: 42.5)
    recorder.start()
    recorder._sources[0]._poll()
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'Float'
    assert message == {'value': 42.5}


def test_scalar_extract_pulls_field(mcap_dir: Path) -> None:
    class _State:
        curvature = 0.25

    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/driver/curvature', event=event,
                    converter=scalar(extract=lambda s: s.curvature))
    recorder.start()
    event.emit(_State())
    recorder.stop()

    _, message = _read(_only_file(mcap_dir))[0]
    assert message == {'value': 0.25}


def test_bool_dispatches_to_boolean(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    add_timer_topic(recorder, '/flag', interval=0.1, read=lambda: True)
    recorder.start()
    recorder._sources[0]._poll()
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'Boolean'
    assert message == {'value': True}


def test_scalar_encodes_numpy_types(mcap_dir: Path) -> None:
    # Regression: np.bool_/np.int64 are not JSON-serializable by default (and in
    # numpy 2.x np.bool_ even reports its type name as 'bool'); coerce via .item().
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    bool_event, int_event = FakeEvent(), FakeEvent()
    add_event_topic(recorder, '/driver/backward', event=bool_event, converter=scalar(value_type='boolean'))
    add_event_topic(recorder, '/count', event=int_event, converter=scalar(value_type='integer'))
    recorder.start()
    bool_event.emit(np.bool_(True))
    int_event.emit(np.int64(7))
    recorder.stop()

    by_topic = {schema_name: message for schema_name, message in _read(_only_file(mcap_dir))}
    assert by_topic['Boolean'] == {'value': True}
    assert by_topic['Integer'] == {'value': 7}


def test_scalars_of_different_types_get_distinct_schema_names(mcap_dir: Path) -> None:
    # Regression: Foxglove keys datatypes by name, so float and bool scalars must
    # not share a single 'Scalar' schema name (it collides as float64 vs bool).
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    number_event, bool_event = FakeEvent(), FakeEvent()
    add_event_topic(recorder, '/driver/curvature', event=number_event, converter=scalar())
    add_event_topic(recorder, '/driver/backward', event=bool_event,
                    converter=scalar(value_type='boolean'))
    recorder.start()
    number_event.emit(0.25)
    bool_event.emit(True)
    recorder.stop()

    schema_names = {name for name, _ in _read(_only_file(mcap_dir))}
    assert schema_names == {'Float', 'Boolean'}


# --------------------------------------------------------------------------- #
# Lifecycle: sources only active while recording
# --------------------------------------------------------------------------- #


def test_subscription_only_active_while_recording(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/pose', event=event)

    assert event.callbacks == []
    recorder.start()
    assert len(event.callbacks) == 1
    recorder.stop()
    assert event.callbacks == []


def test_source_added_while_recording_starts_immediately(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.start()
    event = FakeEvent()
    add_event_topic(recorder, '/pose', event=event)
    assert len(event.callbacks) == 1
    recorder.stop()
    assert event.callbacks == []


def test_messages_after_stop_are_ignored(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/pose', event=event)
    recorder.start()
    recorder.stop()
    event.emit(Pose(x=1.0, y=1.0, yaw=0.0, time=1.0))
    assert recorder._message_count == 0


# --------------------------------------------------------------------------- #
# Timer source
# --------------------------------------------------------------------------- #


def test_timer_topic_polls_and_records(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    values: list[Any] = [Velocity(linear=0.3, angular=0.0, time=1.0)]
    add_timer_topic(recorder, '/wheels', interval=0.1, read=lambda: values[0])
    recorder.start()
    source = recorder._sources[0]
    source._poll()
    values[0] = None  # read() returning None must be skipped
    source._poll()
    recorder.stop()

    assert [m['linear'] for _, m in _read(_only_file(mcap_dir))] == [0.3]


def test_timer_source_repeater_lifecycle(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    add_timer_topic(recorder, '/wheels', interval=0.1, read=lambda: None)
    source = recorder._sources[0]
    assert source._repeater is None
    recorder.start()
    assert source._repeater is not None
    recorder.stop()
    assert source._repeater is None


# --------------------------------------------------------------------------- #
# Converter encoding
# --------------------------------------------------------------------------- #


def test_pose_in_frame_maps_yaw_to_quaternion() -> None:
    data = pose_in_frame(frame='map').encode(Pose(x=1.0, y=2.0, yaw=math.pi, time=0.0), 5 * NS)
    message = json.loads(data)
    assert message['pose']['orientation']['z'] == pytest.approx(1.0)
    assert message['pose']['orientation']['w'] == pytest.approx(0.0, abs=1e-9)
    assert message['timestamp'] == {'sec': 5, 'nsec': 0}


def test_location_fix_converts_radians_to_degrees() -> None:
    class _Pose:
        lat = math.radians(48.5)
        lon = math.radians(9.1)

    class _Measurement:
        pose = _Pose()
        altitude = 412.0
        latitude_std_dev = 0.0
        longitude_std_dev = 0.0

    message = json.loads(location_fix().encode(_Measurement(), 2 * NS))
    assert message['latitude'] == pytest.approx(48.5)
    assert message['longitude'] == pytest.approx(9.1)
    assert message['altitude'] == 412.0
    assert message['position_covariance_type'] == 0  # no std devs reported -> covariance UNKNOWN


def test_transform_3d_uses_full_rotation() -> None:
    pose = Pose3d(x=1.0, y=2.0, z=3.0, rotation=Rotation.from_euler(0.0, 0.0, math.pi))
    message = json.loads(transform_3d(child='camera_front', parent='base_link').encode(pose, NS))
    assert message['child_frame_id'] == 'camera_front'
    assert message['translation'] == {'x': 1.0, 'y': 2.0, 'z': 3.0}
    assert message['rotation']['z'] == pytest.approx(1.0)


def test_camera_calibration_encode() -> None:
    class _Size:
        width, height = 640, 480

    class _Model:
        value = 'pinhole'

    class _Intrinsics:
        matrix = ((600.0, 0.0, 320.0), (0.0, 600.0, 240.0), (0.0, 0.0, 1.0))
        distortion = (0.1, 0.0, 0.0, 0.0, 0.0)
        size = _Size()
        model = _Model()

    class _Calibration:
        intrinsics = _Intrinsics()

    class _Camera:
        calibration = _Calibration()

    message = json.loads(camera_calibration(_Camera(), frame='camera_front').encode(None, NS))
    assert (message['width'], message['height']) == (640, 480)
    assert message['distortion_model'] == 'plumb_bob'
    assert message['K'][0] == 600.0  # fx
    assert message['K'][2] == 320.0  # cx
    assert message['P'][0] == 600.0
    assert message['frame_id'] == 'camera_front'


def test_camera_calibration_skips_when_uncalibrated() -> None:
    class _Camera:
        calibration = None

    assert camera_calibration(_Camera(), frame='camera_front').encode(None, NS) is None


def test_compressed_image_encode(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/camera/front/image', event=event)
    recorder.start()
    event.emit(Image.create_placeholder('hello', camera_id='front', time=5.0))
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'foxglove.CompressedImage'
    assert message['format'] == 'jpeg'
    assert message['timestamp'] == {'sec': 5, 'nsec': 0}
    assert base64.b64decode(message['data'])[:2] == b'\xff\xd8'  # JPEG magic


def test_image_annotations_from_detections(mcap_dir: Path) -> None:
    image = Image.create_placeholder('x', camera_id='front', time=2.0)
    image._detections['detector'] = Detections(
        boxes=[BoxDetection(category_name='weed', model_name='m', confidence=0.9,
                            x=10.0, y=20.0, width=30.0, height=40.0)],
        points=[PointDetection(category_name='crop', model_name='m', confidence=0.8, x=5.0, y=6.0)],
    )
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/camera/front/annotations', event=event, converter=image_annotations())
    recorder.start()
    event.emit(image)
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'foxglove.ImageAnnotations'
    assert message['points'][0]['type'] == 2  # LINE_LOOP
    assert len(message['points'][0]['points']) == 4
    assert message['circles'][0]['position'] == {'x': 5.0, 'y': 6.0}
    assert {t['text'] for t in message['texts']} == {'weed (0.90)', 'crop (0.80)'}


def test_image_annotations_filters_by_camera_id(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/camera/front/annotations', event=event,
                    converter=image_annotations(camera_id='front'))
    recorder.start()
    event.emit(Image.create_placeholder('x', camera_id='back', time=1.0))  # other camera -> skipped
    recorder.stop()
    assert recorder._message_count == 0


# --------------------------------------------------------------------------- #
# custom_message
# --------------------------------------------------------------------------- #


def test_custom_message_roundtrip(mcap_dir: Path) -> None:
    converter = custom_message(
        'TestDiag',
        {'type': 'object', 'properties': {'value': {'type': 'number'}}},
        lambda v, _ts: {'value': v['value']},
    )
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    add_timer_topic(recorder, '/diag', interval=0.1, read=lambda: {'value': 7.5}, converter=converter)
    recorder.start()
    recorder._sources[0]._poll()
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'TestDiag'
    assert message == {'value': 7.5}


def test_custom_message_skips_on_none(mcap_dir: Path) -> None:
    converter = custom_message('TestSkip', {'type': 'object'}, lambda _v, _ts: None)
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    add_timer_topic(recorder, '/skip', interval=0.1, read=lambda: {'value': 1.0}, converter=converter)
    recorder.start()
    recorder._sources[0]._poll()
    recorder.stop()
    assert recorder._message_count == 0


# --------------------------------------------------------------------------- #
# Battery -> sensor_msgs/msg/BatteryState
# --------------------------------------------------------------------------- #


def test_battery_maps_bms_state_to_ros_battery_state(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/battery', event=event,
                    timestamp=lambda state: state.last_update, converter=battery_state())
    recorder.start()
    event.emit(SimpleNamespace(percentage=87.5, voltage=25.2, current=-3.1,
                               temperature=24.0, is_charging=False, last_update=5.0))
    recorder.stop()

    with open(_only_file(mcap_dir), 'rb') as f:
        (schema, _, message), = make_reader(f).iter_messages()
    assert schema.name == 'sensor_msgs/msg/BatteryState'
    assert message.log_time == 5 * NS  # timestamp comes from last_update, not rosys.time()
    payload = json.loads(message.data)
    assert payload['header']['stamp'] == {'sec': 5, 'nanosec': 0}
    assert payload['voltage'] == 25.2
    assert payload['current'] == -3.1
    assert payload['temperature'] == 24.0
    assert payload['percentage'] == pytest.approx(0.875)  # 0..100 rescaled to 0..1
    assert payload['power_supply_status'] == 2  # DISCHARGING
    assert payload['present'] is True


def test_battery_charging_status(mcap_dir: Path) -> None:
    message = json.loads(battery_state().encode(
        SimpleNamespace(percentage=50.0, voltage=24.0, current=2.0, temperature=20.0, is_charging=True), NS))
    assert message['power_supply_status'] == 1  # CHARGING


def test_battery_unmeasured_fields_are_null(mcap_dir: Path) -> None:
    message = json.loads(battery_state().encode(
        SimpleNamespace(percentage=None, voltage=24.0, current=None, temperature=None, is_charging=None), NS))
    assert message['percentage'] is None
    assert message['current'] is None
    assert message['temperature'] is None
    assert message['power_supply_status'] == 0  # UNKNOWN
    assert message['charge'] is None and message['capacity'] is None


def test_battery_auto_dispatches_for_bms_state(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/battery', event=event, timestamp=lambda state: state.last_update)
    recorder.start()
    state = BmsState(percentage=42.0, voltage=24.0, current=-1.0, temperature=21.0,
                     is_charging=False, last_update=3.0)
    event.emit(state)
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'sensor_msgs/msg/BatteryState'
    assert message['percentage'] == pytest.approx(0.42)


# --------------------------------------------------------------------------- #
# GNSS status (custom message)
# --------------------------------------------------------------------------- #


def test_gnss_status_records_diagnostics_with_heading(mcap_dir: Path) -> None:
    """A dual-antenna fix records quality, satellites, hdop, heading and std devs."""
    measurement = GnssMeasurement(
        time=5.0, gnss_time=5.0,
        pose=GeoPose.from_degrees(lat=48.5, lon=9.1, heading=90.0),
        latitude_std_dev=0.01, longitude_std_dev=0.02, heading_std_dev=0.5,
        gps_quality=GpsQuality.RTK_FIXED, num_satellites=18, hdop=0.7,
    )
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/gnss/status', event=event, converter=gnss_status())
    recorder.start()
    event.emit(measurement)
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'GnssStatus'
    assert message['quality'] == 'RTK_FIXED'
    assert message['num_satellites'] == 18
    assert message['hdop'] == pytest.approx(0.7)
    assert message['heading'] == pytest.approx(math.radians(90.0))
    assert message['heading_available'] is True
    assert message['heading_std_dev'] == pytest.approx(0.5)
    assert message['latitude_std_dev'] == pytest.approx(0.01)
    assert message['longitude_std_dev'] == pytest.approx(0.02)


def test_gnss_status_without_heading_stays_valid_json(mcap_dir: Path) -> None:
    """A single-antenna fix (infinite heading std dev) records valid JSON with heading unavailable."""
    measurement = GnssMeasurement(
        time=1.0, gnss_time=1.0,
        pose=GeoPose.from_degrees(lat=48.5, lon=9.1, heading=0.0),
        latitude_std_dev=0.01, longitude_std_dev=0.01, heading_std_dev=float('inf'),
        gps_quality=GpsQuality.RTK_FLOAT, num_satellites=12, hdop=1.1,
    )
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/gnss/status', event=event, converter=gnss_status())
    recorder.start()
    event.emit(measurement)
    recorder.stop()

    raw, = _read_raw(_only_file(mcap_dir))
    assert b'Infinity' not in raw and b'NaN' not in raw  # strict JSON has neither
    message = json.loads(raw)
    assert message['heading_available'] is False
    assert message['heading_std_dev'] is None
    assert message['quality'] == 'RTK_FLOAT'


def test_location_fix_includes_diagonal_covariance_when_std_devs_present() -> None:
    """Known horizontal std devs become a diagonal position covariance (m²) with type DIAGONAL_KNOWN."""
    measurement = GnssMeasurement(
        time=1.0, gnss_time=1.0,
        pose=GeoPose.from_degrees(lat=48.0, lon=9.0, heading=0.0),
        latitude_std_dev=0.02, longitude_std_dev=0.03, altitude=100.0,
    )
    message = json.loads(location_fix().encode(measurement, NS))
    covariance = message['position_covariance']
    assert message['position_covariance_type'] == 2  # DIAGONAL_KNOWN
    assert covariance[0] == pytest.approx(0.03 ** 2)  # East (longitude) variance
    assert covariance[4] == pytest.approx(0.02 ** 2)  # North (latitude) variance
    assert covariance[8] == 0.0  # Up variance unknown (GnssMeasurement has no altitude std dev)
    assert covariance[1] == 0.0 and covariance[3] == 0.0  # off-diagonal stays zero


# --------------------------------------------------------------------------- #
# IMU (custom message)
# --------------------------------------------------------------------------- #


def test_imu_records_rotation_rates_and_calibration(mcap_dir: Path) -> None:
    """The imu converter records roll/pitch/yaw, angular velocities and gyro calibration."""
    measurement = ImuMeasurement(
        time=4.0, gyro_calibration=2.5,
        rotation=Rotation.from_euler(0.1, 0.2, 0.3),
        angular_velocity=Rotation.from_euler(0.01, 0.02, 0.03),
    )
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event = FakeEvent()
    add_event_topic(recorder, '/imu', event=event)
    recorder.start()
    event.emit(measurement)
    recorder.stop()

    schema_name, message = _read(_only_file(mcap_dir))[0]
    assert schema_name == 'ImuMeasurement'
    assert (message['roll'], message['pitch'], message['yaw']) == \
        pytest.approx((0.1, 0.2, 0.3))
    assert (message['angular_velocity_roll'], message['angular_velocity_pitch'],
            message['angular_velocity_yaw']) == pytest.approx((0.01, 0.02, 0.03))
    assert message['gyro_calibration'] == pytest.approx(2.5)


# --------------------------------------------------------------------------- #
# Timestamp fallback and off-loop encoding
# --------------------------------------------------------------------------- #


def test_timestamp_ns_uses_positive_time_attribute() -> None:
    """A payload's positive ``time`` attribute is used as the log time."""
    assert converters._timestamp_ns(SimpleNamespace(time=5.0)) == 5 * NS


def test_timestamp_ns_falls_back_without_usable_time(monkeypatch: pytest.MonkeyPatch) -> None:
    """A payload without ``time`` (or with ``time == 0``) falls back to ``rosys.time()``."""
    monkeypatch.setattr(converters.rosys, 'time', lambda: 7.0)
    assert converters._timestamp_ns(object()) == 7 * NS  # no ``time`` attribute at all
    assert converters._timestamp_ns(SimpleNamespace(time=0.0)) == 7 * NS  # ``time == 0`` treated as unset


def test_deselected_topic_is_not_encoded(mcap_dir: Path) -> None:
    """A deselected topic skips encoding entirely; encoding runs only for recorded topics."""
    encoded: list[Any] = []

    def counting_encode(value: Any, _timestamp_ns: int) -> bytes:
        encoded.append(value)
        return json.dumps({'value': value}).encode()

    converter = Converter(TopicSchema('Counted', b'{}', 'jsonschema', 'json'), counting_encode)
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    kept, dropped = FakeEvent(), FakeEvent()
    add_event_topic(recorder, '/kept', event=kept, converter=converter)
    add_event_topic(recorder, '/dropped', event=dropped, converter=converter)
    recorder.start(topics=['/kept'])
    kept.emit(1)
    dropped.emit(2)  # deselected -> must never be encoded
    recorder.stop()

    assert encoded == [1]


def test_event_subscription_survives_client_deletion(mcap_dir: Path) -> None:
    """A recording started from a UI context keeps recording after that browser tab closes.

    nicegui auto-unsubscribes callbacks subscribed inside a slot context when the client is
    deleted; the recorder opts out (``unsubscribe_on_delete=False``) so event-driven topics
    are not silently lost when the tab that started the recording disconnects.
    """
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    event: Event = Event()
    add_event_topic(recorder, '/pose', event=event)
    client = Client(page('/'))
    with client:  # subscribe from within a UI/client context (as the developer panel does)
        recorder.start()
    assert client.delete_handlers == []  # no auto-unsubscribe registered against the client
    client.delete()  # the browser tab that started the recording disconnects

    event.emit(Pose(x=3.0, y=0.0, yaw=0.0, time=1.0))  # subscription must have survived
    recorder.stop()

    _, message = _read(_only_file(mcap_dir))[0]
    assert message['pose']['position']['x'] == 3.0
