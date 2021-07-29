from ..world.point import Point
from ..world.spline import Spline
from ..world.world import World
from ..actors.esp import Esp
from .spline import drive_spline


async def drive_path(world: World, esp: Esp):

    for spline in world.path:
        await drive_to(world, esp, spline.start)
        await drive_spline(spline, world, esp)


async def drive_to(world: World, esp: Esp, target: Point):

    robot_pose = world.robot.prediction
    approach_spline = Spline(
        start=robot_pose.point,
        control1=robot_pose.point.interpolate(target, 1/3),
        control2=robot_pose.point.interpolate(target, 2/3),
        end=target,
    )
    await drive_spline(approach_spline, world, esp)
