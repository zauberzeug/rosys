from ..hardware import Hardware
from ..world import Point, World
from . import drive_to


async def drive_square(world: World, hardware: Hardware):
    start_pose = world.robot.prediction.copy()
    for x, y in [(1, 0), (1, 1), (0, 1), (0, 0)]:
        await drive_to(world, hardware, start_pose.transform(Point(x=x, y=y)))
