import warnings
from collections.abc import AsyncGenerator

from ... import rosys
from ..camera_provider import CameraProvider
from .simulated_camera import SimulatedCamera


class SimulatedCameraProvider(CameraProvider[SimulatedCamera]):
    """This module collects and simulates cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    """
    SCAN_INTERVAL = 10

    def __init__(self, *,
                 simulate_failing: bool = False,
                 auto_scan: bool = True) -> None:
        super().__init__()

        self.simulate_failing = simulate_failing

        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    @property
    def simulate_device_failure(self) -> bool:
        warnings.warn('`simulate_device_failure` is deprecated, use `simulate_failing` instead.',
                      DeprecationWarning, stacklevel=2)
        return self.simulate_failing

    @simulate_device_failure.setter
    def simulate_device_failure(self, value: bool) -> None:
        warnings.warn('`simulate_device_failure` is deprecated, use `simulate_failing` instead.',
                      DeprecationWarning, stacklevel=2)
        self.simulate_failing = value

    def backup_to_dict(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {}

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = SimulatedCamera.from_dict(camera_data)
            self.add_camera(camera)

    async def scan_for_cameras(self) -> AsyncGenerator[str, None]:
        """Simulated device discovery by returning all camera IDs."""
        for camera in self._cameras.values():
            yield camera.id

    async def update_device_list(self) -> None:
        async for camera_id in self.scan_for_cameras():
            camera = self._cameras.get(camera_id)
            if not camera:
                camera = SimulatedCamera(id=camera_id, width=640, height=480,
                                         simulate_failing=self.simulate_failing)
                self.add_camera(camera)
            if not camera.is_active:
                await camera.connect()

    def add_cameras(self, num_cameras: int) -> None:
        for _ in range(num_cameras):
            new_id = f'cam{len(self._cameras)}'
            self.add_camera(SimulatedCamera(id=new_id, width=640, height=480,
                                            simulate_failing=self.simulate_failing))
