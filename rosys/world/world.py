from pydantic import BaseModel, PrivateAttr
from typing import List, Dict
from enum import Enum
import time
from .mode import Mode
from .robot import Robot
from .marker import Marker
from .camera import Camera
from .image import Image
from .spline import Spline


class WorldState(Enum):

    RUNNING = 1
    PAUSED = 2


class NozzleState(Enum):

    OFF = 0
    ON = 1
    PULSING = 2


class World(BaseModel):

    mode: Mode
    state: WorldState
    _time: float = PrivateAttr(default_factory=time.time)
    robot: Robot
    marker: Marker
    cameras: Dict[str, Camera] = {}
    download_queue: List[str] = []
    images: List[Image] = []
    image_data: Dict[str, bytes] = {}
    path: List[Spline] = []
    nozzle: NozzleState = NozzleState.OFF

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value
