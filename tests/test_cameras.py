import asyncio
import platform

import pytest

from rosys.testing import forward
from rosys.vision import RtspCamera, RtspCameraProvider, SimulatedCamera, UsbCamera, UsbCameraProvider


async def test_simulated_camera(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1)
    await camera.connect()
    assert camera.is_connected
    await forward(1.1)
    assert len(camera.images) >= 1
    assert camera.images[0].size.width == 800
    assert camera.images[0].size.height == 600


async def test_usb_camera(rosys_integration):
    if platform.system() != 'Linux':
        pytest.skip('UsbCamera is only supported on Linux.')
    connected_uids = list(await UsbCameraProvider.scan_for_cameras())
    if len(connected_uids) == 0:
        pytest.skip('No USB camera detected. This test requires a physical USB camera to be connected.')
    camera = UsbCamera(id=connected_uids[0])
    await camera.connect()
    await asyncio.sleep(0.5)
    assert camera.is_connected
    assert len(camera.images) >= 1


async def test_rtsp_camera(rosys_integration):
    try:
        connected_uids = await RtspCameraProvider.scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No RTSP camera detected. This test requires a physical RTSP camera on the local network.')
    mac, ip = connected_uids[0]
    camera = RtspCamera(id=mac, ip=ip)
    await camera.connect()
    await asyncio.sleep(0.5)
    assert camera.is_connected
    await asyncio.sleep(1)
    assert len(camera.images) >= 1
