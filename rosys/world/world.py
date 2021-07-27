from pydantic import BaseModel, PrivateAttr
from typing import List, Dict, Optional
from enum import Enum
import time
from .mode import Mode
from .robot import Robot
from .marker import Marker
from .camera import Camera
from .image import Image
from .link import Link
from .spline import Spline
from .pose import Pose


class WorldState(Enum):

    RUNNING = 1
    PAUSED = 2


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
    link_queue: List[List[str]] = []
    links: List[Link] = []
    path: List[Spline] = []
    carrot: Optional[Pose]

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value
