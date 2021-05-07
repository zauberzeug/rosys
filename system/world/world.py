from pydantic import BaseModel
from typing import List, Dict
from world.state import State
from world.mode import Mode
from world.robot import Robot
from world.camera import Camera
from world.image import Image


class World(BaseModel):

    mode: Mode
    state: State
    time: float
    robot: Robot
    cameras: Dict[str, Camera]
    images: List[Image]
    image_data: Dict[str, bytes]
