from ..world.world import World
from ..world.spline import Spline
from ..actors.esp import Esp
from .spline import drive_spline


async def draw_path(world: World, esp: Esp):

    for spline_ in world.path:
        robot_pose = world.robot.prediction
        if robot_pose.point.distance(spline_.start) > 0.01:
            await drive_spline(Spline(
                start=robot_pose.point,
                control1=robot_pose.point.interpolate(spline_.start, 1/3),
                control2=robot_pose.point.interpolate(spline_.start, 2/3),
                end=spline_.start,
            ), world, esp)
        await drive_spline(spline_, world, esp)
