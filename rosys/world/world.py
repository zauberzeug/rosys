from pydantic import BaseModel, PrivateAttr
import time
from .. import is_test
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

    @property
    def time(self) -> float:
        return self._time if is_test else time.time()

    def set_time(self, value):
        assert is_test
        self._time = value

    @property
    def cameras(self) -> dict[str, Camera]:
        return self.usb_cameras
