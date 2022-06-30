from ..hardware import Wheels
from ..world import Point, World
from . import drive_to


async def drive_square(world: World, wheels: Wheels):
    start_pose = world.robot.prediction.copy()
    for x, y in [(1, 0), (1, 1), (0, 1), (0, 0)]:
        await drive_to(world, wheels, start_pose.transform(Point(x=x, y=y)))
