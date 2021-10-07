from rosys.world.upload import Upload
from rosys.world.obstacle import Obstacle
from pydantic import BaseModel, PrivateAttr
from typing import Union
from enum import Enum
import time
from .camera import Camera
from .image import Image
from .link import Link
from .mode import Mode
from .path_segment import PathSegment
from .robot import Robot


class WorldState(Enum):
    RUNNING = 1
    PAUSED = 2


class World(BaseModel):
    mode: Mode
    state: WorldState = WorldState.PAUSED
    _time: float = PrivateAttr(default_factory=time.time)
    robot: Robot
    cameras: dict[str, Camera] = {}
    tracking: Union[bool, list[str]] = False
    download_queue: list[str] = []
    upload: Upload = Upload()
    images: list[Image] = []
    image_data: dict[str, bytes] = {}
    link_queue: list[list[str]] = []
    links: list[Link] = []
    path: list[PathSegment] = []
    obstacles: dict[str, Obstacle] = {}

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value

    def get_captured_images(self, camera: Camera) -> Image:
        return [i for i in self.images if i.mac == camera.mac and i.id in self.image_data]

    def get_camera(self, image: Image):
        return self.cameras.get(image.mac)

    def sees_robot(self, camera: Camera):
        latest = [i for i in self.get_captured_images(camera) if i.time > self.time - 1]
        return any([i.marker_points for i in latest])
