import asyncio
import shutil

import pytest

from rosys.vision import GmslCamera, GmslCameraProvider
from rosys.vision.gmsl_camera.gmsl_camera_scanner import count_gmsl_sensors, parse_sensor_ids
from rosys.vision.gmsl_camera.gmsl_device import build_argus_command
from rosys.vision.gstreamer import parse_caps_dimensions

# Representative `v4l2-ctl --list-devices` output of a Jetson GMSL carrier board with a single camera connected.
# A sensor exposes several `/dev/video` nodes, and the media controller / a dummy device must be ignored.
V4L2_LIST_DEVICES_OUTPUT = '''\
vi-output, gmsl-cam 0-0015 (platform:tegra-capture-vi:2):
\t/dev/video4
\t/dev/video5

NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
\t/dev/media0

dummy video (platform:vivid-000):
\t/dev/video10
\t/dev/video11
'''

# Three GMSL sensors connected (the planned multi-camera setup), each exposing two video nodes.
V4L2_LIST_DEVICES_THREE_SENSORS = '''\
NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
\t/dev/media0

vi-output, gmsl-cam 0-0011 (platform:tegra-capture-vi:0):
\t/dev/video0
\t/dev/video1

vi-output, gmsl-cam 0-0015 (platform:tegra-capture-vi:2):
\t/dev/video4
\t/dev/video5

vi-output, gmsl-cam 0-0013 (platform:tegra-capture-vi:3):
\t/dev/video2
\t/dev/video3
'''


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


def test_count_one_sensor_ignoring_nodes_and_media_controller() -> None:
    # one sensor with two video nodes -> a single sensor, ids {0}; the media controller and dummy are ignored
    assert count_gmsl_sensors(V4L2_LIST_DEVICES_OUTPUT) == 1
    assert parse_sensor_ids(V4L2_LIST_DEVICES_OUTPUT) == {0}


def test_count_three_sensors() -> None:
    assert count_gmsl_sensors(V4L2_LIST_DEVICES_THREE_SENSORS) == 3
    assert parse_sensor_ids(V4L2_LIST_DEVICES_THREE_SENSORS) == {0, 1, 2}


def test_parse_sensor_ids_empty_without_tegra_devices() -> None:
    assert parse_sensor_ids('dummy video (platform:vivid-000):\n\t/dev/video10\n') == set()


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
    """Hardware test: requires a Jetson with a connected GMSL camera and a working Argus stack."""
    if not GmslCameraProvider.is_operable():
        pytest.skip('gst-launch-1.0 is not installed. This test requires a Jetson with the Argus GStreamer stack.')
    sensor_ids = await GmslCameraProvider.scan_for_cameras()
    if not sensor_ids:
        pytest.skip('No GMSL sensor detected. This test requires a physical GMSL camera on a Jetson.')
    camera = GmslCamera(id='gmsl-test', sensor_id=min(sensor_ids), connect_after_init=False)
    await camera.connect()
    await asyncio.sleep(2.0)
    try:
        assert camera.is_connected
        assert len(camera.images) >= 1
    finally:
        await camera.disconnect()
