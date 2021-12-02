from pydantic import BaseModel, PrivateAttr
from enum import Enum
import time
from .mode import Mode
from .obstacle import Obstacle
from .path_segment import PathSegment
from .robot import Robot


class AutomationState(Enum):
    RUNNING = 1
    PAUSED = 2


class World(BaseModel):
    __slots__ = ['__weakref__']  # required for nicegui binding with weakrefs

    robot: Robot = Robot()
    mode: Mode = Mode.REAL
    automation_state: AutomationState = AutomationState.PAUSED
    _time: float = PrivateAttr(default_factory=time.time)
    tracking: bool = False
    robot_locator_cam: str = None
    download_queue: list[str] = []
    image_data: dict[str, bytes] = {}
    link_queue: list[list[str]] = []
    path: list[PathSegment] = []
    obstacles: dict[str, Obstacle] = {}
    notifications: list[tuple[float, str]] = []

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value
