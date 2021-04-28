from pydantic import BaseModel
from typing import Dict
from world.state import State
from world.mode import Mode
from world.robot import Robot
from world.camera import Camera


class World(BaseModel):

    mode: Mode
    state: State
    time: float
    robot: Robot
    cameras: Dict[str, Camera]
