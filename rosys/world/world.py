from pydantic import BaseModel, PrivateAttr
from enum import Enum
import time
from .mode import Mode
from .obstacle import Obstacle
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
    tracking: bool = False
    robot_locator_cam: str = None
    download_queue: list[str] = []
    image_data: dict[str, bytes] = {}
    link_queue: list[list[str]] = []
    path: list[PathSegment] = []
    obstacles: dict[str, Obstacle] = {}

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value
