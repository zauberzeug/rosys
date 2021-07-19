from ..world.world import World
from ..world.spline import Spline
from ..actors.esp import Esp
from .spline import drive_spline


async def draw_path(world: World, esp: Esp):

    for spline in world.path:
        robot_pose = world.robot.prediction
        approach_spline = Spline(
            start=robot_pose.point,
            control1=robot_pose.point.interpolate(spline.start, 1/3),
            control2=robot_pose.point.interpolate(spline.start, 2/3),
            end=spline.start,
        )
        await drive_spline(approach_spline, world, esp)
        await drive_spline(spline, world, esp)
