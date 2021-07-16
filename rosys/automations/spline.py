import numpy as np
from ..actors.esp import Esp
from ..world.world import World
from ..world.mode import Mode
from ..world.point import Point
from ..world.spline import Spline
from .navigation.carrot import Carrot


def eliminate_2pi(angle):

    return (angle + np.pi) % (2 * np.pi) - np.pi


def eliminate_pi(angle):

    return (angle + np.pi / 2) % np.pi - np.pi / 2


def ramp(x: float, in_min: float, in_max: float, out_min: float, out_max: float, clip: bool = False):

    if clip and x < in_min:
        return out_min
    if clip and x > in_max:
        return out_max
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


async def spline(spline: Spline, world: World, esp: Esp):

    carrot = Carrot(spline=spline)

    linear_limit = world.robot.parameters.linear_speed_limit
    angular_limit = world.robot.parameters.angular_speed_limit

    is_offset = world.robot.shape.point_of_interest.distance(Point(x=0, y=0)) > 0

    while True:

        point_of_interest = world.robot.prediction.transform(world.robot.shape.point_of_interest)
        if not carrot.move(point_of_interest):
            break

        if is_offset:
            direction_poi_to_carrot = point_of_interest.direction(carrot.pose.point)
            turn_angle = eliminate_pi(direction_poi_to_carrot - world.robot.prediction.yaw)
            # NOTE: rectangular triangle with heigth h and hypothenuse segments p, q
            h = world.robot.shape.point_of_interest.x
            p = np.tan(turn_angle) * h
            q = h**2 / p
            r = world.robot.shape.point_of_interest.y + q
            curvature = 1 / r
            rotation_center = world.robot.prediction.transform(Point(x=0, y=r))
            carrot_sign = eliminate_2pi(
                rotation_center.direction(point_of_interest) -
                rotation_center.direction(carrot.pose.point)
            ) > 0
            robot_sign = eliminate_2pi(
                rotation_center.direction(point_of_interest) -
                rotation_center.direction(world.robot.prediction.point)
            ) > 0
            backward = bool(carrot_sign == robot_sign) != bool(h < 0)
        else:
            local_spline = Spline.from_poses(world.robot.prediction, carrot.pose)
            curvature = local_spline.max_curvature(0.0, 0.25)
            backward = False

        age = world.time - world.robot.detection.time
        linear = 0.5
        if world.mode != Mode.TEST:  # TODO: require camera tracking in tests as well
            linear *= ramp(age, 3, 6, 1.0, 0.0, clip=True)
        linear *= -1 if backward else 1
        if carrot.t > 1.0:
            linear *= ramp(carrot.target_distance, 0.5, 0.0, 1.0, 0.5)
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
