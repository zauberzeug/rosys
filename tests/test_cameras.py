import platform

import pytest

from rosys.vision import RtspCamera, RtspCameraProvider, SimulatedCameraProvider, UsbCamera, UsbCameraProvider


async def test_simulated_camera():
    camera = SimulatedCameraProvider.create('test_cam', width=800, height=600)
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1
    assert camera.images[0].size.width == 800
    assert camera.images[0].size.height == 600


async def test_usb_camera():
    if platform.system() != 'Linux':
        pytest.skip('UsbCamera is only supported on Linux.')
    connected_uids = await UsbCameraProvider.scan_for_cameras()
    if len(connected_uids) == 0:
        pytest.skip('No USB camera detected. This test requires a physical USB camera to be connected.')
    camera = UsbCamera(id=connected_uids[0])
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1


async def test_rtsp_camera():
    try:
        connected_uids = await RtspCameraProvider.scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No RTSP camera detected. This test requires a physical RTSP camera on the local network.')
    camera = RtspCamera(id=connected_uids[0])
    await camera.connect()
    assert camera.is_connected
    await camera.capture_image()
    assert len(camera.images) == 1
