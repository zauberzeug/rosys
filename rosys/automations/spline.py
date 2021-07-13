from ..actors.esp import Esp
from ..world.world import World
from ..world.spline import Spline
from .navigation.carrot import Carrot


async def spline(spline: Spline, world: World, esp: Esp):

    carrot = Carrot(spline)

    linear_limit = world.robot.parameters.linear_speed_limit
    angular_limit = world.robot.parameters.angular_speed_limit

    while carrot.move(world.robot.prediction):

        local_spline = Spline.from_poses(world.robot.prediction, carrot.pose)
        curvature = local_spline.max_curvature(0.0, 0.25)
        linear = 0.5
        angular = linear * curvature

        if abs(linear) > linear_limit:
            factor = linear_limit / abs(linear)
            angular *= factor
            linear *= factor
        if abs(angular) > angular_limit:
            factor = angular_limit / abs(angular)
            linear *= factor
            angular *= factor

        await esp.drive(linear, angular)

    await esp.drive(0, 0)
