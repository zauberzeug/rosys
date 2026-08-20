import asyncio
from contextlib import asynccontextmanager

import numpy as np
import pytest
import pytest_asyncio

from rosys.vision import Image, MjpegCamera, MjpegCameraProvider
from rosys.vision.mjpeg_camera.mjpeg_device import MjpegDevice
from rosys.vision.mjpeg_camera.motec_mjpeg_device import MotecMjpegDevice
from rosys.vision.mjpeg_camera.vendors import VendorType, mac_to_vendor

PART_HEADER = b'--boundary\r\nContent-Type: image/jpeg\r\n\r\n'
GOODCAM_MAC = '2c:6f:51:00:00:01'  # a vendor whose device needs no network to configure


def _mjpeg_stream(frames: int = 3) -> bytes:
    noise = np.random.default_rng(seed=0).integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
    return (PART_HEADER + Image.from_array(noise).to_jpeg_bytes()) * frames


class _StubResponse:
    def __init__(self, stream: bytes, chunk_size: int) -> None:
        self._chunks = [stream[i:i + chunk_size] for i in range(0, len(stream), chunk_size)]

    async def aiter_bytes(self):
        for chunk in self._chunks:
            yield chunk


async def test_mjpeg_camera(rosys_integration):
    try:
        connected_uids = await MjpegCameraProvider().scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No MJPEG camera detected. This test requires a physical MJPEG camera on the local network.')
    uid, ip = connected_uids[0]
    camera = MjpegCamera(id=uid, ip=ip, connect_after_init=False)
    await camera.connect()
    await asyncio.sleep(0.5)
    assert camera.is_connected
    assert len(camera.images) >= 1


@pytest_asyncio.fixture()
async def motec_settings_interface(rosys_integration):
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
            camera = MjpegCamera(id=mac, connect_after_init=False, ip=ip)
            await camera.connect()
            break
    else:
        pytest.skip('No MOTEC camera detected. This test requires a physical MOTEC camera on the local network.')

    assert camera.device is not None
    assert isinstance(camera.device, MotecMjpegDevice)
    assert camera.device.settings_interface is not None
    yield camera.device


async def test_fps(motec_settings_interface):
    fps = await motec_settings_interface.get_fps()
    try:
        await motec_settings_interface.set_fps(30)
        assert await motec_settings_interface.get_fps() == 30
        await motec_settings_interface.set_fps(10)
        assert await motec_settings_interface.get_fps() == 10
    finally:
        await motec_settings_interface.set_fps(fps)


async def test_stream_compression(motec_settings_interface):
    compression_level = await motec_settings_interface.get_stream_compression()
    try:
        await motec_settings_interface.set_stream_compression(1)
        assert await motec_settings_interface.get_stream_compression() == 1
        await motec_settings_interface.set_stream_compression(4)
        assert await motec_settings_interface.get_stream_compression() == 4
    finally:
        await motec_settings_interface.set_stream_compression(compression_level)


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


async def test_stream_port(motec_settings_interface):
    port = await motec_settings_interface.get_stream_port()
    try:
        await motec_settings_interface.set_stream_port(8885)
        assert await motec_settings_interface.get_stream_port() == 8885
        await motec_settings_interface.set_stream_port(8886)
        assert await motec_settings_interface.get_stream_port() == 8886
    finally:
        await motec_settings_interface.set_stream_port(port)


@pytest.mark.parametrize('chunk_size', [4096, 1_000_000])
async def test_transferred_bytes_counts_every_byte_read(chunk_size, monkeypatch):
    """Everything the stream delivers is counted, however it is chunked and however many frames survive."""
    stream = _mjpeg_stream()
    camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1', connect_after_init=False)
    camera.device = MjpegDevice(mac=camera.mac, ip='127.0.0.1',
                                on_new_image_data=camera._handle_new_image_data)  # pylint: disable=protected-access

    @asynccontextmanager
    async def fake_open_stream(*_, **__):
        yield _StubResponse(stream, chunk_size)
    monkeypatch.setattr('rosys.vision.mjpeg_camera.mjpeg_device.open_stream', fake_open_stream)

    await camera.device.run_capture_task()

    assert camera.transferred_bytes == len(stream)
    assert len(camera.images) <= 3  # frames may be skipped to stay current, bytes are not
    assert camera.images[-1].byte_size() != camera.transferred_bytes  # traffic, not decoded size


async def test_transferred_bytes_survives_a_reconnect():
    """Reconnecting replaces the device, so the camera keeps the retired one's bytes to stay monotonic."""
    camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1', connect_after_init=False)
    assert camera.transferred_bytes == 0

    await camera.connect()
    assert camera.device is not None
    camera.device.transferred_bytes = 1000
    assert camera.transferred_bytes == 1000

    await camera.disconnect()
    assert camera.transferred_bytes == 1000

    await camera.connect()
    assert camera.device is not None
    assert camera.device.transferred_bytes == 0  # the new device starts from zero
    camera.device.transferred_bytes = 7
    assert camera.transferred_bytes == 1007
