import json
import random
import tempfile
from pathlib import Path

import pytest
from mcap.reader import make_reader

from rosys.recording import McapLogger

NS = 1_000_000_000


@pytest.fixture
def mcap_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield Path(tmp)


def test_write_and_read_messages(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'value': {'type': 'number'}},
    })
    logger.start()

    for i in range(5):
        logger.log_message('/test', {'value': i}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    assert logger._message_count == 5
    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) == 1

    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [(json.loads(msg.data), msg.log_time) for _, _, msg in reader.iter_messages()]
    assert len(messages) == 5
    assert messages[0][0] == {'value': 0}
    assert messages[4][0] == {'value': 4}
    assert messages[2][1] == 2_000_000_000


def test_file_rotation(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, max_file_size_mb=0.001, auto_start=False)
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'payload': {'type': 'string'}},
    })
    logger.start()

    large_payload = 'x' * 500
    for i in range(20):
        logger.log_message('/test', {'payload': large_payload}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) >= 2, f'Expected rotation, got {len(files)} file(s)'


def test_disk_budget_enforcement(mcap_dir: Path) -> None:
    logger = McapLogger(
        output_dir=mcap_dir,
        max_file_size_mb=0.001,
        max_total_size_mb=0.003,
        auto_start=False,
    )
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'payload': {'type': 'string'}},
    })
    logger.start()

    large_payload = 'x' * 500
    for i in range(100):
        logger.log_message('/test', {'payload': large_payload}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    total = sum(f.stat().st_size for f in mcap_dir.glob('*.mcap'))
    max_allowed = 0.003 * 1_048_576 + 0.001 * 1_048_576  # budget + one active file
    assert total <= max_allowed, f'Total {total} exceeds budget+active {max_allowed}'


def test_unknown_topic_ignored(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.start()

    logger.log_message('/nonexistent', {'value': 1}, timestamp_ns=0)

    logger.stop()
    assert logger._message_count == 0


def test_add_topic_while_recording(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.start()

    logger.add_topic('/late', schema_name='LateMessage', schema={
        'type': 'object',
        'properties': {'value': {'type': 'number'}},
    })
    logger.log_message('/late', {'value': 42}, timestamp_ns=0)
    logger.stop()

    assert logger._message_count == 1
    files = list(mcap_dir.glob('*.mcap'))
    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [json.loads(msg.data) for _, _, msg in reader.iter_messages()]
    assert messages == [{'value': 42}]


def test_compression_options(mcap_dir: Path) -> None:
    for comp in ('zstd', 'lz4', 'none'):
        subdir = mcap_dir / comp
        subdir.mkdir()
        logger = McapLogger(output_dir=subdir, compression=comp, auto_start=False)
        logger.add_topic('/test', schema_name='Test', schema={
            'type': 'object',
            'properties': {'v': {'type': 'number'}},
        })
        logger.start()
        logger.log_message('/test', {'v': 1}, timestamp_ns=0)
        logger.stop()

        files = list(subdir.glob('*.mcap'))
        assert len(files) == 1, f'compression={comp} failed'
        with open(files[0], 'rb') as f:
            reader = make_reader(f)
            messages = [json.loads(msg.data) for _, _, msg in reader.iter_messages()]
        assert messages == [{'v': 1}]


def test_stop_without_start(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.stop()
    assert not logger.is_recording
    assert list(mcap_dir.glob('*.mcap')) == []


def test_double_start(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.add_topic('/test', schema_name='Test', schema={
        'type': 'object',
        'properties': {'v': {'type': 'number'}},
    })
    logger.start()
    first_file = logger._file_path
    logger.start()
    assert logger._file_path == first_file
    logger.stop()


def test_simulated_sensor_recording_30s(mcap_dir: Path) -> None:
    """Simulate 30s of realistic sensor data (GNSS 1Hz, IMU 10Hz, Wheels 50Hz) and report file size."""
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)

    gnss_schema = {
        'type': 'object',
        'properties': {
            'latitude_deg': {'type': 'number'}, 'longitude_deg': {'type': 'number'},
            'heading_deg': {'type': 'number'}, 'latitude_std_dev': {'type': 'number'},
            'longitude_std_dev': {'type': 'number'}, 'heading_std_dev': {'type': 'number'},
            'gps_quality': {'type': 'integer'}, 'gps_quality_name': {'type': 'string'},
            'num_satellites': {'type': 'integer'}, 'hdop': {'type': 'number'},
            'altitude': {'type': 'number'},
        },
    }
    imu_schema = {
        'type': 'object',
        'properties': {
            'roll': {'type': 'number'}, 'pitch': {'type': 'number'}, 'yaw': {'type': 'number'},
            'angular_velocity_roll': {'type': 'number'},
            'angular_velocity_pitch': {'type': 'number'},
            'angular_velocity_yaw': {'type': 'number'},
            'gyro_calibration': {'type': 'number'},
        },
    }
    velocity_schema = {
        'type': 'object',
        'properties': {
            'linear': {'type': 'number'},
            'angular': {'type': 'number'},
        },
    }

    logger.add_topic('/gnss', schema_name='GnssMeasurement', schema=gnss_schema)
    logger.add_topic('/imu', schema_name='ImuMeasurement', schema=imu_schema)
    logger.add_topic('/wheels/measured', schema_name='WheelVelocityMeasured', schema=velocity_schema)
    logger.add_topic('/wheels/commanded', schema_name='WheelVelocityCommanded', schema=velocity_schema)
    logger.start()

    lat, lon, heading = 48.123456, 9.654321, 90.0
    roll, pitch, yaw = 0.01, -0.02, 1.57
    lin_vel, ang_vel = 0.5, 0.02

    duration_ms = 30_000
    for ms in range(duration_ms):
        t_ns = ms * 1_000_000

        if ms % 1000 == 0:
            lat += random.gauss(0, 0.000001)
            lon += random.gauss(0, 0.000001)
            heading += random.gauss(0, 0.1)
            logger.log_message('/gnss', {
                'latitude_deg': lat, 'longitude_deg': lon, 'heading_deg': heading,
                'latitude_std_dev': 0.012, 'longitude_std_dev': 0.015, 'heading_std_dev': 0.8,
                'gps_quality': 4, 'gps_quality_name': 'RTK_FIXED',
                'num_satellites': 18, 'hdop': 0.7, 'altitude': 412.345,
            }, timestamp_ns=t_ns)

        if ms % 100 == 0:
            roll += random.gauss(0, 0.001)
            pitch += random.gauss(0, 0.001)
            yaw += random.gauss(0, 0.005)
            logger.log_message('/imu', {
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'angular_velocity_roll': random.gauss(0, 0.01),
                'angular_velocity_pitch': random.gauss(0, 0.01),
                'angular_velocity_yaw': random.gauss(0, 0.05),
                'gyro_calibration': 3.0,
            }, timestamp_ns=t_ns)

        if ms % 20 == 0:
            logger.log_message('/wheels/measured', {
                'linear': lin_vel + random.gauss(0, 0.01),
                'angular': ang_vel + random.gauss(0, 0.005),
            }, timestamp_ns=t_ns)
            logger.log_message('/wheels/commanded', {
                'linear': lin_vel + random.gauss(0, 0.001),
                'angular': ang_vel + random.gauss(0, 0.001),
            }, timestamp_ns=t_ns)

    logger.stop()

    expected_gnss = duration_ms // 1000
    expected_imu = duration_ms // 100
    expected_wheels = (duration_ms // 20) * 2
    expected_total = expected_gnss + expected_imu + expected_wheels
    assert logger._message_count == expected_total

    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) == 1
    file_size = files[0].stat().st_size

    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        topics: dict[str, int] = {}
        for _, channel, _ in reader.iter_messages():
            topics[channel.topic] = topics.get(channel.topic, 0) + 1

    assert topics['/gnss'] == expected_gnss
    assert topics['/imu'] == expected_imu
    assert topics['/wheels/measured'] == expected_wheels // 2
    assert topics['/wheels/commanded'] == expected_wheels // 2

    print(f'\n--- 30s simulated recording ---')
    print(f'  Messages: {logger._message_count}')
    print(f'  File size: {file_size / 1024:.1f} KB')
    print(f'  -> 10 min extrapolated: {file_size * 20 / 1024:.1f} KB ({file_size * 20 / 1_048_576:.2f} MB)')
    print(f'  -> 1 hour extrapolated: {file_size * 120 / 1_048_576:.1f} MB')
    print(f'  -> 8 hours extrapolated: {file_size * 960 / 1_048_576:.1f} MB')
    print(f'  Per topic: {topics}')
