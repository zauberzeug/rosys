from rosys.world.obstacle import Obstacle
from pydantic import BaseModel, PrivateAttr
from typing import Optional, Union
from enum import Enum
import time
from .camera import Camera
from .image import Image
from .link import Link
from .marker import Marker
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
    marker: Optional[Marker]
    cameras: dict[str, Camera] = {}
    tracking: Union[bool, list[str]] = False
    download_queue: list[str] = []
    upload_queue: list[str] = []
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
