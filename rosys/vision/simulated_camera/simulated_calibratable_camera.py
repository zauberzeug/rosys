from ..camera import CalibratableCamera
from .simulated_camera import SimulatedCamera


class SimulatedCalibratableCamera(SimulatedCamera, CalibratableCamera):
    pass
