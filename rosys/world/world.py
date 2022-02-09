from pydantic import BaseModel, PrivateAttr
import time
from .area import Area
from .camera import Camera
from .mode import Mode
from .obstacle import Obstacle
from .robot import Robot
from .upload import Upload
from .usb_camera import UsbCamera


class World(BaseModel):
    robot: Robot = Robot()
    mode: Mode = Mode.REAL
    _time: float = PrivateAttr(default_factory=time.time)
    obstacles: dict[str, Obstacle] = {}
    areas: dict[str, Area] = {}
    notifications: list[tuple[float, str]] = []
    usb_cameras: dict[str, UsbCamera] = {}
    upload: Upload = Upload()
    needs_backup: bool = False

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value

    @property
    def cameras(self) -> dict[str, Camera]:
        return self.usb_cameras
