from ..actors.esp import Esp
from ..world.world import World
from ..world.point import Point
from .drive_path import drive_to


async def drive_square(world: World, esp: Esp):
    start_pose = world.robot.prediction.copy()
    for x, y in [(1, 0), (1, 1), (0, 1), (0, 0)]:
        await drive_to(world, esp, start_pose.transform(Point(x=x, y=y)))
