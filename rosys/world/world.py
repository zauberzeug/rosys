from pydantic import BaseModel
from typing import List, Dict
from .state import State
from .mode import Mode
from .robot import Robot
from .marker import Marker
from .camera import Camera
from .image import Image
from .spline import Spline


class World(BaseModel):

    mode: Mode
    state: State
    time: float
    robot: Robot
    marker: Marker
    cameras: Dict[str, Camera] = {}
    download_queue: List[str] = []
    images: List[Image] = []
    image_data: Dict[str, bytes] = {}
    path: List[Spline] = []
