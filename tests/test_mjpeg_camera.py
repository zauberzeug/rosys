import asyncio

import pytest
import pytest_asyncio

from rosys.vision import MjpegCamera, MjpegCameraProvider
from rosys.vision.mjpeg_camera.vendors import VendorType, mac_to_vendor

# from ..rosys.vision.mjpeg_camera.motec_settings_interface import MotecSettingsInterface


@pytest.fixture(scope='session')
def event_loop():
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.mark.asyncio
async def test_mjpeg_camera():
    try:
        connected_uids = await MjpegCameraProvider().scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No MJPEG camera detected. This test requires a physical MJPEG camera on the local network.')
    camera = MjpegCamera(id=connected_uids[0][0], connect_after_init=False, streaming=False)
    await camera.connect(ip=connected_uids[0][1])
    await asyncio.sleep(0.5)
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1


@pytest_asyncio.fixture(scope='session')
async def motec_settings_interface():
    try:
        connected_uids = await MjpegCameraProvider().scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No MJPEG camera detected. This test requires a physical MJPEG camera on the local network.')

    for mac, ip in connected_uids:
        if mac_to_vendor(mac) == VendorType.MOTEC:
            camera = MjpegCamera(id=mac, connect_after_init=False)
            await camera.connect(ip=ip)
            break
    else:
        pytest.skip('No MOTEC camera detected. This test requires a physical MOTEC camera on the local network.')

    return camera.device.settings_interface


@pytest.mark.asyncio
async def test_fps(motec_settings_interface):
    fps = await motec_settings_interface.get_fps()
    try:
        await motec_settings_interface.set_fps(30)
        assert await motec_settings_interface.get_fps() == 30
        await motec_settings_interface.set_fps(10)
        assert await motec_settings_interface.get_fps() == 10
    finally:
        await motec_settings_interface.set_fps(fps)


@pytest.mark.asyncio
async def test_stream_compression(motec_settings_interface):
    compression_level = await motec_settings_interface.get_stream_compression()
    try:
        await motec_settings_interface.set_stream_compression(1)
        assert await motec_settings_interface.get_stream_compression() == 1
        await motec_settings_interface.set_stream_compression(4)
        assert await motec_settings_interface.get_stream_compression() == 4
    finally:
        await motec_settings_interface.set_stream_compression(compression_level)


@pytest.mark.asyncio
async def test_stream_resolution(motec_settings_interface):
    resolution = await motec_settings_interface.get_stream_resolution()
    try:
        await motec_settings_interface.set_stream_resolution(1920, 1080)
        val = await motec_settings_interface.get_stream_resolution()
        assert val == (1920, 1080)
        await motec_settings_interface.set_stream_resolution(480, 360)
        assert await motec_settings_interface.get_stream_resolution() == (480, 360)
    finally:
        await motec_settings_interface.set_stream_resolution(*resolution)


@pytest.mark.asyncio
async def test_stream_port(motec_settings_interface):
    port = await motec_settings_interface.get_stream_port()
    try:
        await motec_settings_interface.set_stream_port(8885)
        assert await motec_settings_interface.get_stream_port() == 8885
        await motec_settings_interface.set_stream_port(8886)
        assert await motec_settings_interface.get_stream_port() == 8886
    finally:
        await motec_settings_interface.set_stream_port(port)