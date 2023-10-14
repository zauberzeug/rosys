import pytest

from rosys.vision import (Camera, RtspCamera, RtspCameraProvider, SimulatedCamera, SimulatedCameraProvider, UsbCamera,
                          UsbCameraProvider)


async def test_simulated_camera():
    camera = SimulatedCameraProvider.create('test_cam', width=800, height=600)
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1
    assert camera.images[0].size.width == 800
    assert camera.images[0].size.height == 600


async def test_usb_camera():
    connected_uids = await UsbCameraProvider.scan_for_cameras()
    if len(connected_uids) == 0:
        pytest.skip('No USB camera detected. This test requires a physical USB camera to be connected.')
    camera = UsbCamera(id=connected_uids[0])
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1


async def test_rtsp_camera():
    connected_uids = await RtspCameraProvider.scan_for_cameras()
    if len(connected_uids) == 0:
        pytest.skip('No RTSP camera detected. This test requires a physical RTSP camera on the local network.')
    camera = RtspCamera(id=connected_uids[0])
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1
