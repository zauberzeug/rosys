import random
from collections.abc import AsyncGenerator
from typing import Any

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

        self.simulate_device_failure = simulate_failing

        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    def backup_to_dict(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {}

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = SimulatedCamera.from_dict(camera_data)
            self.add_camera(camera)

        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    async def scan_for_cameras(self) -> AsyncGenerator[str, Any]:
        """Simulated device discovery by returning all camera's IDs.

        If simulate_device_failure is set, disconnected cameras are returned with a fixed probability.
        """
        for camera in self._cameras.values():
            if not camera.is_connected and self.simulate_device_failure:
                # return camera with fixed probability
                if random.random() < 0.5:
                    yield camera.id
            else:
                yield camera.id

    async def update_device_list(self) -> None:
        async for camera_id in self.scan_for_cameras():
            camera = self._cameras.get(camera_id)
            if not camera:
                camera = SimulatedCamera(id=camera_id, width=640, height=480)
                self.add_camera(camera)

            if not camera.is_connected:
                await camera.connect()
                continue
            assert camera.device is not None

            if self.simulate_device_failure:
                # disconnect cameras randomly with probability rising with time
                time_since_last_activation = rosys.time() - camera.device.creation_time
                if random.random() < time_since_last_activation / 30.0:
                    await camera.disconnect()

    def add_cameras(self, num_cameras: int) -> None:
        for _ in range(num_cameras):
            new_id = f'cam{len(self._cameras)}'
            self.add_camera(SimulatedCamera(id=new_id, width=640, height=480))
