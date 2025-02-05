import asyncio
import platform

import pytest

from rosys.geometry import Rectangle
from rosys.testing import forward
from rosys.vision import (
    CalibratableCamera,
    Calibration,
    ConfigurableCamera,
    MjpegCamera,
    RtspCamera,
    RtspCameraProvider,
    SimulatedCamera,
    TransformableCamera,
    UsbCamera,
    UsbCameraProvider,
)


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


async def test_storing_simulated_camera_as_dict(rosys_integration):
    camera = SimulatedCamera(id='test_cam', name='T3:5T', connect_after_init=False,
                             width=808, height=606, color='#010101', fps=1)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = SimulatedCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, SimulatedCamera)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.connect_after_init == camera.connect_after_init
    assert restored_camera.resolution == camera.resolution
    assert restored_camera.parameters == camera.parameters


async def test_storing_configurable_camera_as_dict(rosys_integration):
    camera = ConfigurableCamera(id='test_cam', name='T3:5T', connect_after_init=False,
                                base_path_overwrite='test_cam', image_history_length=10)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = ConfigurableCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, ConfigurableCamera)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.parameters == camera.parameters
    assert restored_camera.connect_after_init == camera.connect_after_init
    assert restored_camera.base_path == camera.base_path
    assert restored_camera.images.maxlen == camera.images.maxlen


def test_storing_transformable_camera_as_dict(rosys_integration):
    camera = TransformableCamera(id='test_cam', name='T3:5T',
                                 crop=Rectangle(x=10, y=10, width=100, height=100),
                                 rotation=90)
    camera_as_dict = camera.to_dict()
    restored_camera = TransformableCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, TransformableCamera)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.crop == camera.crop
    assert restored_camera.rotation == camera.rotation
    assert restored_camera.rotation_angle == camera.rotation_angle


def test_storing_calibratable_camera_as_dict(rosys_integration):
    camera = CalibratableCamera.create_calibrated(id='test_cam', width=808, height=606,
                                                  focal_length=577, x=1.0, y=2.0, z=3.0)
    assert isinstance(camera.calibration, Calibration)
    camera_as_dict = camera.to_dict()
    restored_camera = CalibratableCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, CalibratableCamera)
    assert isinstance(restored_camera.calibration, Calibration)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.calibration == camera.calibration


async def test_storing_mjpeg_camera_as_dict(rosys_integration):
    camera = MjpegCamera(id='test_cam', name='T3:5T', connect_after_init=False,
                         base_path_overwrite='test_cam', ip='192.168.1.1',
                         username='admin', password='admin', fps=1,
                         resolution=(808, 606), mirrored=True)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = MjpegCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, MjpegCamera)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.parameters == camera.parameters
    assert restored_camera.connect_after_init == camera.connect_after_init
    assert restored_camera.username == camera.username
    assert restored_camera.password == camera.password
    assert restored_camera.ip == camera.ip
    assert restored_camera.mac == camera.mac


def test_storing_rtsp_camera_as_dict(rosys_integration):
    camera = RtspCamera(id='test_cam', name='T3:5T', connect_after_init=False,
                        fps=1, substream=2, bitrate=4097, ip='192.168.1.1')
    camera_as_dict = camera.to_dict()
    restored_camera = RtspCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, RtspCamera)
    assert restored_camera.id == camera.id
    assert restored_camera.name == camera.name
    assert restored_camera.parameters == camera.parameters
    assert restored_camera.ip == camera.ip
