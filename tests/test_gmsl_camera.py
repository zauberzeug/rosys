import asyncio
import os
import shutil

import pytest

from rosys.vision import GmslCamera, GmslCameraProvider
from rosys.vision.gmsl_camera.gmsl_device import build_argus_command
from rosys.vision.gstreamer import parse_caps_dimensions


def test_build_command_auto_by_default() -> None:
    command = build_argus_command(5)
    assert 'nvarguscamerasrc sensor-id=5' in command
    assert 'exposuretimerange' not in command
    assert 'gainrange' not in command
    assert 'gdppay ! fdsink' in command
    assert 'format=RGB' in command


def test_build_command_pins_manual_exposure_in_nanoseconds() -> None:
    command = build_argus_command(0, auto_exposure=False, exposure=0.25)
    assert 'exposuretimerange="250000000 250000000"' in command


def test_build_command_pins_manual_gain() -> None:
    command = build_argus_command(0, auto_gain=False, gain=4.0)
    assert 'gainrange="4.0 4.0"' in command


def test_build_command_sets_resolution_and_framerate() -> None:
    command = build_argus_command(0, width=1920, height=1200, fps=4)
    assert 'width=1920,height=1200,framerate=4/1' in command


def test_parse_caps_dimensions() -> None:
    caps = 'video/x-raw, format=(string)RGB, width=(int)1920, height=(int)1200, framerate=(fraction)30/1'
    assert parse_caps_dimensions(caps) == (1920, 1200)


def test_to_dict_round_trip() -> None:
    camera = GmslCamera(id='gmsl-5', sensor_id=5, connect_after_init=False,
                        auto_exposure=False, exposure=0.25, fps=4)
    data = camera.to_dict()
    assert data['sensor_id'] == 5
    assert data['exposure'] == 0.25
    restored = GmslCamera.from_dict(data)
    assert restored.sensor_id == 5
    assert restored.parameters['exposure'] == 0.25
    assert restored.parameters['fps'] == 4


def test_provider_not_operable_without_gstreamer() -> None:
    if shutil.which('gst-launch-1.0') is not None:
        pytest.skip('gst-launch-1.0 is installed; cannot test the non-operable case')
    assert GmslCameraProvider.is_operable() is False


async def test_gmsl_camera_capture(rosys_integration):
    """Hardware test: requires a Jetson with a connected GMSL camera and a working Argus stack.

    Set ``GMSL_TEST_SENSOR_ID`` to the Argus sensor-id of a connected camera (default 0).
    """
    if not GmslCameraProvider.is_operable():
        pytest.skip('gst-launch-1.0 is not installed. This test requires a Jetson with the Argus GStreamer stack.')
    sensor_id = int(os.environ.get('GMSL_TEST_SENSOR_ID', '0'))
    camera = GmslCamera(id='gmsl-test', sensor_id=sensor_id, connect_after_init=False)
    await camera.connect()
    await asyncio.sleep(3.0)
    try:
        if not camera.images:
            pytest.skip(f'No frames from sensor-id {sensor_id}; requires a physical GMSL camera on a Jetson.')
        assert camera.images[-1].size.width > 0
    finally:
        await camera.disconnect()
