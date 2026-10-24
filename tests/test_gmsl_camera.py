import asyncio
import os
import shutil
from unittest.mock import AsyncMock, patch

import pytest

import rosys
from rosys.testing import forward
from rosys.vision import GmslCamera, GmslCameraProvider
from rosys.vision.gmsl_camera.gmsl_device import GmslDevice, build_argus_command
from rosys.vision.gstreamer import parse_caps_dimensions, read_tail


def gstreamer_available():
    """Let `GmslCamera.connect()` proceed on a machine without the Argus GStreamer stack."""
    return patch('shutil.which', return_value='/usr/bin/gst-launch-1.0')


def test_build_command_auto_by_default() -> None:
    command = build_argus_command(5)
    assert 'nvarguscamerasrc sensor-id=5' in command
    assert 'exposuretimerange' not in command
    assert 'gainrange' not in command
    assert 'ispdigitalgainrange' not in command
    assert 'gdppay ! fdsink' in command
    assert 'format=RGB' in command


def test_build_command_pins_manual_exposure_in_nanoseconds() -> None:
    command = build_argus_command(0, auto_exposure=False, exposure=0.25)
    assert 'exposuretimerange="250000000 250000000"' in command


def test_build_command_pins_manual_gain_and_isp_digital_gain() -> None:
    command = build_argus_command(0, auto_gain=False, gain=4.0)
    assert 'gainrange="4.0 4.0"' in command
    assert 'ispdigitalgainrange="1 1"' in command, 'expected the ISP not to compensate the pinned analog gain'


def test_build_command_sets_resolution_and_framerate() -> None:
    command = build_argus_command(0, width=1920, height=1200, fps=4)
    assert 'width=1920,height=1200,framerate=4/1' in command


def test_parse_caps_dimensions() -> None:
    caps = 'video/x-raw, format=(string)RGB, width=(int)1920, height=(int)1200, framerate=(fraction)30/1'
    assert parse_caps_dimensions(caps) == (1920, 1200)


async def test_read_tail_keeps_the_last_bytes_until_eof() -> None:
    stream = asyncio.StreamReader()
    stream.feed_data(b'x' * 10_000 + b'ERROR: no camera')
    stream.feed_eof()
    assert await read_tail(stream, 20) == b'xxxxERROR: no camera'


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


def test_id_defaults_to_sensor_id() -> None:
    assert GmslCamera(sensor_id=3, connect_after_init=False).id == 'gmsl-3'
    assert GmslCamera(id='front', sensor_id=3, connect_after_init=False).id == 'front'


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


async def test_parameters_set_before_the_first_frame_reach_the_device(rosys_integration):
    first_frame = asyncio.Event()

    async def session(self: GmslDevice) -> None:
        await first_frame.wait()
        await self._enter_streaming()  # pylint: disable=protected-access
        await rosys.sleep(60.0)

    camera = GmslCamera(sensor_id=0, connect_after_init=False)
    with gstreamer_available(), patch.object(GmslDevice, '_run_session', session):
        await camera.connect()
        try:
            assert camera.device is not None
            assert not camera.is_connected, 'expected no connection before the first frame'
            await camera.set_parameters({'exposure': 0.25})
            first_frame.set()
            await forward(until=lambda: camera.is_connected)
            assert camera.device.exposure == 0.25, 'expected the parameter to reach the device once it streams'
        finally:
            await camera.disconnect()


async def _connected_session(self: GmslDevice) -> None:
    """Stand-in for a pipeline that delivers frames right away but spawns nothing."""
    await self._enter_streaming()  # pylint: disable=protected-access
    await rosys.sleep(60.0)


async def test_reapplying_parameters_on_connect_does_not_restart_the_pipeline(rosys_integration):
    camera = GmslCamera(sensor_id=0, connect_after_init=False)
    with gstreamer_available(), \
            patch.object(GmslDevice, '_run_session', _connected_session), \
            patch.object(GmslDevice, 'restart_gstreamer', new_callable=AsyncMock) as restart:
        await camera.connect()
        try:
            await forward(until=lambda: camera.is_connected)
            await forward(1.0)  # give a debounced restart the chance to run
            restart.assert_not_called()

            await camera.set_parameters({'fps': 7})
            await forward(1.0)
            restart.assert_called_once()
        finally:
            await camera.disconnect()
