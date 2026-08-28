import warnings

from ..camera_provider import CameraProvider
from .simulated_camera import SimulatedCamera


class SimulatedCameraProvider(CameraProvider[SimulatedCamera]):
    """This module collects and simulates cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    """

    def __init__(self, *, simulate_failing: bool = False) -> None:
        super().__init__()

        self.simulate_failing = simulate_failing

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

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = SimulatedCamera.from_dict(camera_data)
            self.add_camera(camera)

    async def update_device_list(self) -> None:
        """Simulated cameras are created explicitly with `add_cameras`, so there is nothing to discover."""

    def add_cameras(self, num_cameras: int) -> None:
        for _ in range(num_cameras):
            new_id = f'cam{len(self._cameras)}'
            self.add_camera(SimulatedCamera(id=new_id, width=640, height=480,
                                            simulate_failing=self.simulate_failing))
