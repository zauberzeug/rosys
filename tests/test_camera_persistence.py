import pytest

from rosys.geometry import Rectangle
from rosys.vision import (
    CalibratableCamera,
    Calibration,
    Camera,
    MjpegCamera,
    RtspCamera,
    SimulatedCamera,
    TransformableCamera,
    UsbCamera,
)


@pytest.fixture
def base_camera_parameters() -> dict:
    return {
        'id': 'test_cam',
        'name': 'T3:5T',
        'connect_after_init': False,
        'base_path_overwrite': 'new_base_path',
        'image_history_length': 10
    }


def assert_base_camera_parameters_match(camera1: Camera, camera2: Camera) -> None:
    assert camera1.id == camera2.id
    assert camera1.name == camera2.name
    assert camera1.connect_after_init == camera2.connect_after_init
    assert camera1.base_path == camera2.base_path
    assert camera1.images.maxlen == camera2.images.maxlen


async def test_storing_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = Camera(**base_camera_parameters)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = Camera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, Camera)
    assert_base_camera_parameters_match(camera, restored_camera)


async def test_storing_camera_with_no_basepath_overwrite(rosys_integration, base_camera_parameters):
    camera = Camera(id='test_cam', base_path_overwrite=None)
    camera_as_dict = camera.to_dict()
    restored_camera = Camera.from_dict(camera_as_dict)
    assert camera_as_dict['base_path_overwrite'] is None
    assert isinstance(restored_camera, Camera)
    assert restored_camera.base_path == camera.base_path


async def test_storing_simulated_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = SimulatedCamera(width=808, height=606, color='#010101', fps=1, **base_camera_parameters)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = SimulatedCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, SimulatedCamera)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.resolution == camera.resolution
    assert restored_camera.parameters == camera.parameters


def test_storing_transformable_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = TransformableCamera(crop=Rectangle(x=10, y=10, width=100, height=100),
                                 rotation=90, **base_camera_parameters)
    camera_as_dict = camera.to_dict()
    restored_camera = TransformableCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, TransformableCamera)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.crop == camera.crop
    assert restored_camera.rotation == camera.rotation
    assert restored_camera.rotation_angle == camera.rotation_angle


async def test_storing_mjpeg_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = MjpegCamera(ip='192.168.1.1', username='admin', password='admin', fps=1,
                         resolution=(808, 606), mirrored=True, **base_camera_parameters)
    await camera.connect()
    camera_as_dict = camera.to_dict()
    restored_camera = MjpegCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, MjpegCamera)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.parameters == camera.parameters
    assert restored_camera.username == camera.username
    assert restored_camera.password == camera.password
    assert restored_camera.ip == camera.ip
    assert restored_camera.mac == camera.mac


def test_storing_rtsp_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = RtspCamera(fps=1, substream=2, bitrate=4097, h265=True, ip='192.168.1.1', **base_camera_parameters)
    camera_as_dict = camera.to_dict()
    restored_camera = RtspCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, RtspCamera)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.parameters == camera.parameters
    assert restored_camera.ip == camera.ip


def test_storing_usb_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = UsbCamera(auto_exposure=False, exposure=100, width=808, height=606, fps=1, **base_camera_parameters)
    camera_as_dict = camera.to_dict()
    restored_camera = UsbCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, UsbCamera)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.parameters == camera.parameters


def test_storing_calibratable_camera_as_dict(rosys_integration, base_camera_parameters):
    camera = CalibratableCamera.create_calibrated(width=808, height=606,
                                                  focal_length=577, x=1.0, y=2.0, z=3.0,
                                                  **base_camera_parameters)
    assert isinstance(camera.calibration, Calibration)
    camera_as_dict = camera.to_dict()
    restored_camera = CalibratableCamera.from_dict(camera_as_dict)
    assert isinstance(restored_camera, CalibratableCamera)
    assert isinstance(restored_camera.calibration, Calibration)
    assert_base_camera_parameters_match(camera, restored_camera)
    assert restored_camera.calibration == camera.calibration


def test_storing_calibratable_camera_subclasses_as_dict(rosys_integration):
    calibration = CalibratableCamera.create_calibrated(id='test_cam', width=808, height=606, focal_length=577,
                                                       x=1.0, y=2.0, z=3.0).calibration
    assert calibration is not None

    class CalibratableRtspCamera(CalibratableCamera, RtspCamera):
        pass

    class CalibratableUsbCamera(CalibratableCamera, UsbCamera):
        pass

    class CalibratableMjpegCamera(CalibratableCamera, MjpegCamera):
        pass

    class CalibratableSimulatedCamera(CalibratableCamera, SimulatedCamera):
        pass

    for camera_class in [CalibratableRtspCamera, CalibratableUsbCamera, CalibratableMjpegCamera, CalibratableSimulatedCamera]:
        camera = camera_class(id='test_cam')
        camera.calibration = calibration
        camera_as_dict = camera.to_dict()
        restored_camera = camera_class.from_dict(camera_as_dict)
        assert isinstance(restored_camera, camera_class)
        assert restored_camera.id == camera.id
        assert restored_camera.calibration == camera.calibration
