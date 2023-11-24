import random

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from .simulated_camera import SimulatedCamera


class SimulatedCameraProvider(CameraProvider[SimulatedCamera], persistence.PersistentModule):
    """This module collects and simulates cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    """

    def __init__(self, simulate_failing: bool = False) -> None:
        super().__init__()

        self.simulate_device_failure = simulate_failing

        rosys.on_repeat(self.simulate_device_discovery, 5.0)

    def backup(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {}

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = SimulatedCamera.from_dict(camera_data)
            self.add_camera(camera)

        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    async def simulate_device_discovery(self) -> None:
        for camera in self._cameras.values():
            if not camera.is_connected:
                await camera.connect()
                continue
            assert camera.device is not None

            if self.simulate_device_failure:
                # disconnect cameras randomly with probability rising with time
                time_since_last_activation = rosys.time() - camera.device.creation_time
                if random.random() < time_since_last_activation / 30.0:
                    camera.device = None

    def add_cameras(self, num_cameras: int) -> None:
        for _ in range(num_cameras):
            new_id = f'cam{len(self._cameras)}'
            print(f'adding simulated camera: {new_id}')
            self.add_camera(SimulatedCamera(id=new_id, width=640, height=480))
