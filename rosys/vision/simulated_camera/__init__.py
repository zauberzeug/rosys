from ..camera import CalibratableCamera
from .simulated_camera import SimulatedCamera
from .simulated_camera_provider import SimulatedCameraProvider


class SimulatedCalibratableCamera(SimulatedCamera, CalibratableCamera):
    pass


__all__ = [
    'SimulatedCamera',
    'SimulatedCameraProvider',
    'SimulatedCalibratableCamera',
]
