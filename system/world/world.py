from pydantic import BaseModel
from world.mode import Mode
from world.robot import Robot


class World(BaseModel):

    mode: Mode
    time: float
    robot: Robot
