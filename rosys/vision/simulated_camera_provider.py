import io
import random
from typing import Any, Optional

import numpy as np

from rosys import persistence

from .. import rosys
from .camera_provider import CameraProvider
from .image import ImageSize
from .simulated_camera import SimulatedCamera


class SimulatedDeviceScanner:
    def __init__(self, n_devices) -> None:
        self.n_devices = n_devices

    async def scan_for_cameras(self) -> list[str]:
        return [f'cam{i}' for i in range(self.n_devices)]

    def add_device(self) -> None:
        self.n_devices += 1

    def remove_device(self) -> None:
        self.n_devices -= min(0, self.n_devices)


class SimulatedCameraProvider(CameraProvider):
    """This module collects and simulates USB cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    """
    USE_PERSISTENCE: bool = False

    def __init__(self, simulate_failing=False) -> None:
        super().__init__()

        self._cameras: dict[str, SimulatedCamera] = {}

        self.simulate_device_failure = simulate_failing

        rosys.on_repeat(self.simulate_device_discovery, 5.0)

        self.needs_backup: bool = False
        if self.USE_PERSISTENCE:
            persistence.register(self)

    def backup(self) -> dict:
        # TODO store cameras
        return {}

    def restore(self, data: dict[str, dict]) -> None:
        # TODO load cameras
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    async def simulate_device_discovery(self) -> None:
        for camera in self._cameras.values():
            if not camera.is_connected:
                await camera.connect()
                continue
            if self.simulate_device_failure:
                # disconnect cameras rendomly with probabilty rising with time
                time_since_last_activation = rosys.time() - camera.device.creation_time
                if random.random() < time_since_last_activation / 30.:
                    camera.device = None

    def get_next_video_id(self) -> int:
        return max([camera.device.video_id for camera in self._cameras.values() if camera.is_connected], default=0) + 1

    def add_cameras(self, num_cameras: int) -> None:
        for _ in range(num_cameras):
            new_id = f'cam{len(self._cameras)}'
            print(f'adding simulated camera: {new_id}')
            self.add_camera(SimulatedCameraProvider.create(new_id, width=640, height=480))

    @staticmethod
    def create(uid: str, width: int = 800, height: int = 600, color: Optional[str] = None) -> SimulatedCamera:
        color = color or f'#{random.randint(0, 0xffffff):06x}'
        return SimulatedCamera(id=uid, resolution=ImageSize(width=width, height=height), color=color)

    @staticmethod
    def create_calibrated(uid: str, *,
                          width: int = 800, height: int = 600, color: Optional[str] = None,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0) -> SimulatedCamera:
        camera = SimulatedCameraProvider.create(uid, width, height, color)
        camera.set_perfect_calibration(width=width, height=height, x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        return camera
