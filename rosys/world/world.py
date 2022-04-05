import datetime
import time

import humanize
from pydantic import BaseModel, PrivateAttr

from .. import event
from ..helpers import is_test
from .area import Area
from .camera import Camera
from .obstacle import Obstacle
from .robot import Robot
from .upload import Upload
from .usb_camera import UsbCamera


class World(BaseModel):
    robot: Robot = Robot()
    _time: float = PrivateAttr(default_factory=time.time)
    obstacles: dict[str, Obstacle] = {}
    areas: dict[str, Area] = {}
    notifications: list[tuple[float, str]] = []
    usb_cameras: dict[str, UsbCamera] = {}
    upload: Upload = Upload()
    needs_backup: bool = False
    start_time: float = 0

    @property
    def time(self) -> float:
        return self._time if is_test else time.time()

    def set_time(self, value):
        assert is_test
        self._time = value

    @property
    def cameras(self) -> dict[str, Camera]:
        return self.usb_cameras

    @property
    def uptime(self) -> str:
        uptime = datetime.timedelta(seconds=self.time - self.start_time)
        return humanize.precisedelta(uptime)

    @property
    def lizard_offset(self) -> str:
        if self.robot.hardware_time is None:
            return '-'
        offset_ms = (self.time - self.robot.hardware_time) * 1000
        return f'{int(offset_ms):4} ms'

    async def add_usb_camera(self, camera: UsbCamera) -> None:
        self.usb_cameras[camera.id] = camera
        await event.call(event.Id.NEW_CAMERA, camera)
