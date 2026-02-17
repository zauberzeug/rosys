"""Tests for the ImageSequenceRecorder and TimestampIndex."""
import json
from pathlib import Path

import pytest

from rosys.testing import forward
from rosys.vision import SimulatedCamera, SimulatedCameraProvider
from rosys.vision.record_replay.image_sequence_recorder import (
    FrameMetadata,
    ImageSequenceRecorder,
    RecordingSession,
    TimestampIndex,
)


@pytest.mark.usefixtures('rosys_integration')
async def test_start_stop_session(recordings_dir: Path):
    """Test starting and stopping a recording session."""
    provider = SimulatedCameraProvider(auto_scan=False)
    cam = SimulatedCamera(id='cam:0', width=64, height=48, fps=2)
    provider.add_camera(cam)
    await cam.connect()

    recorder = ImageSequenceRecorder(provider, recordings_dir)

    # Start session
    session = recorder.start_session('cam:0')
    assert session is not None
    assert session.camera_id == 'cam:0'
    assert session.output_dir.exists()
    assert 'cam:0' in recorder.active_sessions

    # Record some frames
    await forward(seconds=1.5)

    # Stop session
    completed_session = await recorder.stop_session('cam:0')
    assert completed_session is not None
    assert completed_session.end_time is not None
    assert len(completed_session.frames) >= 1
    assert 'cam:0' not in recorder.active_sessions


@pytest.mark.usefixtures('rosys_integration')
async def test_frame_metadata_recorded(recordings_dir: Path):
    """Test that frame metadata is correctly recorded."""
    provider = SimulatedCameraProvider(auto_scan=False)
    cam = SimulatedCamera(id='cam:1', width=32, height=24, fps=2)
    provider.add_camera(cam)
    await cam.connect()

    recorder = ImageSequenceRecorder(provider, recordings_dir, write_metadata=True)
    session = recorder.start_session('cam:1')

    await forward(seconds=1.5)

    completed = await recorder.stop_session('cam:1')
    assert len(completed.frames) >= 1

    # Check metadata file was written
    metadata_path = session.output_dir / 'metadata.json'
    assert metadata_path.exists()

    # Verify metadata content
    with open(metadata_path) as f:
        data = json.load(f)
    assert data['camera_id'] == 'cam:1'
    assert data['frame_count'] >= 1
    assert len(data['frames']) >= 1

    frame = data['frames'][0]
    assert 'filename' in frame
    assert 'timestamp' in frame
    assert 'frame_index' in frame


@pytest.mark.usefixtures('rosys_integration')
async def test_multiple_sessions(recordings_dir: Path):
    """Test recording from multiple cameras simultaneously."""
    provider = SimulatedCameraProvider(auto_scan=False)
    cam1 = SimulatedCamera(id='cam:A', width=32, height=24, fps=2)
    cam2 = SimulatedCamera(id='cam:B', width=32, height=24, fps=2)
    provider.add_camera(cam1)
    provider.add_camera(cam2)
    await cam1.connect()
    await cam2.connect()

    recorder = ImageSequenceRecorder(provider, recordings_dir)
    recorder.start_session('cam:A')
    recorder.start_session('cam:B')

    assert len(recorder.active_sessions) == 2

    await forward(seconds=1.5)

    sessions = await recorder.stop_all_sessions()
    assert len(sessions) == 2
    assert recorder.active_sessions == []


@pytest.mark.usefixtures('rosys_integration')
async def test_image_files_saved(recordings_dir: Path):
    """Test that image files are actually saved to disk."""
    provider = SimulatedCameraProvider(auto_scan=False)
    cam = SimulatedCamera(id='cam:2', width=32, height=24, fps=2)
    provider.add_camera(cam)
    await cam.connect()

    recorder = ImageSequenceRecorder(provider, recordings_dir)
    session = recorder.start_session('cam:2')

    await forward(seconds=1.5)

    await recorder.stop_session('cam:2')

    # Check that image files exist
    jpg_files = list(session.output_dir.glob('*.jpg'))
    assert len(jpg_files) >= 1


def test_timestamp_index_load(tmp_path: Path):
    """Test loading timestamp index from metadata file."""
    session_dir = tmp_path / 'session'
    session_dir.mkdir()

    # Create test metadata
    metadata = {
        'session_id': 'test_session',
        'camera_id': 'cam:test',
        'start_time': 1000.0,
        'end_time': 1010.0,
        'frame_count': 3,
        'frames': [
            {'filename': 'frame0.jpg', 'timestamp': 1000.0, 'frame_index': 0, 'capture_latency_ms': 5.0},
            {'filename': 'frame1.jpg', 'timestamp': 1001.0, 'frame_index': 1, 'capture_latency_ms': 4.5},
            {'filename': 'frame2.jpg', 'timestamp': 1002.0, 'frame_index': 2, 'capture_latency_ms': 5.2},
        ],
    }
    with open(session_dir / 'metadata.json', 'w') as f:
        json.dump(metadata, f)

    # Load index
    index = TimestampIndex(session_dir)
    assert index.frame_count == 3
    assert index.start_time == 1000.0
    assert index.end_time == 1002.0


def test_timestamp_index_find_by_timestamp(tmp_path: Path):
    """Test finding frames by timestamp."""
    session_dir = tmp_path / 'session'
    session_dir.mkdir()

    metadata = {
        'camera_id': 'cam:test',
        'frames': [
            {'filename': 'frame0.jpg', 'timestamp': 1.0, 'frame_index': 0},
            {'filename': 'frame1.jpg', 'timestamp': 2.0, 'frame_index': 1},
            {'filename': 'frame2.jpg', 'timestamp': 3.0, 'frame_index': 2},
            {'filename': 'frame3.jpg', 'timestamp': 5.0, 'frame_index': 3},
        ],
    }
    with open(session_dir / 'metadata.json', 'w') as f:
        json.dump(metadata, f)

    index = TimestampIndex(session_dir)

    # Exact match
    frame = index.find_by_timestamp(2.0)
    assert frame is not None
    assert frame.filename == 'frame1.jpg'

    # Within tolerance
    frame = index.find_by_timestamp(2.05, tolerance=0.1)
    assert frame is not None
    assert frame.filename == 'frame1.jpg'

    # Outside tolerance
    frame = index.find_by_timestamp(2.5, tolerance=0.1)
    assert frame is None

    # Between frames, returns closest
    frame = index.find_by_timestamp(4.0, tolerance=2.0)
    assert frame is not None
    assert frame.filename in ('frame2.jpg', 'frame3.jpg')


def test_timestamp_index_get_frame_path(tmp_path: Path):
    """Test getting frame path from index."""
    session_dir = tmp_path / 'session'
    session_dir.mkdir()

    metadata = {
        'camera_id': 'cam:test',
        'frames': [
            {'filename': 'test.jpg', 'timestamp': 1.0, 'frame_index': 0},
        ],
    }
    with open(session_dir / 'metadata.json', 'w') as f:
        json.dump(metadata, f)

    index = TimestampIndex(session_dir)
    frame = index.frames[0]
    path = index.get_frame_path(frame)
    assert path == session_dir / 'test.jpg'


def test_frame_metadata_dataclass():
    """Test FrameMetadata dataclass."""
    frame = FrameMetadata(
        filename='test.jpg',
        timestamp=12345.678,
        camera_id='cam:0',
        frame_index=42,
        capture_latency_ms=10.5,
    )
    assert frame.filename == 'test.jpg'
    assert frame.timestamp == 12345.678
    assert frame.camera_id == 'cam:0'
    assert frame.frame_index == 42
    assert frame.capture_latency_ms == 10.5


def test_recording_session_to_dict():
    """Test RecordingSession serialization."""
    session = RecordingSession(
        session_id='test123',
        start_time=1000.0,
        camera_id='cam:test',
        output_dir=Path('/tmp/test'),
    )
    session.frames.append(FrameMetadata(
        filename='frame.jpg',
        timestamp=1000.5,
        camera_id='cam:test',
        frame_index=0,
    ))
    session.end_time = 1010.0

    data = session.to_dict()
    assert data['session_id'] == 'test123'
    assert data['start_time'] == 1000.0
    assert data['end_time'] == 1010.0
    assert data['camera_id'] == 'cam:test'
    assert data['frame_count'] == 1
    assert len(data['frames']) == 1
