import asyncio
from datetime import datetime
from pathlib import Path

import pytest

from rosys.testing import forward
from rosys.vision import SimulatedCamera, SimulatedCameraProvider
from rosys.vision.record_replay.image_recorder import ImageRecorder
from rosys.vision.record_replay.replay_camera import ReplayCamera
from rosys.vision.record_replay.replay_camera_provider import ReplayCameraProvider


@pytest.mark.usefixtures('rosys_integration')
async def test_record_images(recordings_dir: Path):
    # ARRANGE
    provider = SimulatedCameraProvider(auto_scan=False)
    cam = SimulatedCamera(id='cam:0', width=64, height=48, fps=1)
    provider.add_camera(cam)
    await cam.connect()

    recorder = ImageRecorder(provider, recordings_dir)
    recorder.set_recording(True)

    # ACT
    await forward(seconds=1.1)

    # ASSERT
    cam_dir = recordings_dir / 'cam--0'
    assert cam_dir.exists()
    files = sorted(p for p in cam_dir.iterdir() if p.suffix == '.jpg')
    assert len(files) == 1

    recorder.set_recording(False)
    await forward(seconds=2.1)
    files_after = [p for p in cam_dir.iterdir() if p.suffix == '.jpg']
    assert len(files_after) == len(files)


@pytest.mark.usefixtures('rosys_integration')
async def test_playback_replays_images(recordings_dir: Path):
    # ARRANGE
    provider = SimulatedCameraProvider(auto_scan=False)
    cam = SimulatedCamera(id='cam:1', width=32, height=24, fps=1)
    provider.add_camera(cam)
    await cam.connect()
    recorder = ImageRecorder(provider, recordings_dir)
    recorder.set_recording(True)
    await forward(seconds=5.1)
    recorder.set_recording(False)

    # ACT
    rp = ReplayCameraProvider(recordings_dir)
    # ASSERT
    assert rp.cameras, 'no cameras detected in replay folder'

    # ACT
    await forward(seconds=0.1)
    await asyncio.sleep(0.1)  # wait is needed to ensure the image is loaded
    await forward(seconds=0.1)  # second forward to retrieve image from CPU pool

    # ASSERT
    replay_cam = next(iter(rp.cameras.values()))
    assert len(replay_cam.images) == 1
    assert replay_cam.latest_captured_image is not None


def test_find_closest_past_index_unit(tmp_path: Path):
    # build a minimal ReplayCamera with fake files and timestamps
    img_dir = tmp_path / 'camA'
    img_dir.mkdir()
    # create timestamped dummy files
    times = [1.0, 2.0, 2.5, 5.0]
    for t in times:
        p = img_dir / f"{datetime.fromtimestamp(t).strftime('%Y-%m-%d_%H-%M-%S.%f')}.jpg"
        p.write_bytes(b'jpg')

    cam = ReplayCamera(camera_id='camA', images_dir=img_dir, camera_name='camA')
    idx = cam._find_closest_past_index(2.3)
    assert idx == 1
    idx = cam._find_closest_past_index(0.5)
    assert idx == -1
    idx = cam._find_closest_past_index(9.0)
    assert idx == len(times) - 1
