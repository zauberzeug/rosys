import numpy as np
from ..actors.esp import Esp
from ..world.world import World
from ..world.point import Point
from .drive_path import drive_to


async def drive_square(world: World, esp: Esp):
    origin = world.robot.prediction.copy()
    for point in [Point(x=1, y=0), Point(x=1, y=1), Point(x=0, y=1), Point(x=0, y=0)]:
        await drive_to(world, esp, origin.transform(point))
