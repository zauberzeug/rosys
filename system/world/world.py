from pydantic import BaseModel
from world.robot import Robot


class World(BaseModel):

    time: float
    robot: Robot
