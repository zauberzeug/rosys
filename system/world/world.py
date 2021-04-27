from world.state import State
from pydantic import BaseModel
from world.mode import Mode
from world.robot import Robot


class World(BaseModel):

    mode: Mode
    state: State
    time: float
    robot: Robot
