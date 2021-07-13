from pydantic import BaseModel
from typing import List, Dict
from enum import Enum
from .mode import Mode
from .robot import Robot
from .marker import Marker
from .camera import Camera
from .image import Image
from .spline import Spline


class WorldState(Enum):

    RUNNING = 1
    PAUSED = 2


class World(BaseModel):

    mode: Mode
    state: WorldState
    time: float
    robot: Robot
    marker: Marker
    cameras: Dict[str, Camera] = {}
    download_queue: List[str] = []
    images: List[Image] = []
    image_data: Dict[str, bytes] = {}
    path: List[Spline] = []
