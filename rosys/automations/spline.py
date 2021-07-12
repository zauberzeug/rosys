from ..actors.esp import Esp
from ..world.world import World
from ..world.spline import Spline
from .navigation.carrot import Carrot


async def spline(spline: Spline, world: World, esp: Esp):
    carrot = Carrot(spline)

    while carrot.move(world.robot.prediction):
        local_spline = Spline.from_poses(world.robot.prediction, carrot.pose)
        curvature = local_spline.max_curvature(0.0, 0.25)
        linear = 0.5
        angular = linear * curvature
        await esp.drive(linear, angular)

    await esp.drive(0, 0)
