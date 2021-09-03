import numpy as np
from ..actors.esp import Esp
from ..world.world import World
from ..world.mode import Mode
from ..world.point import Point
from ..world.spline import Spline
from ..helpers import eliminate_pi, eliminate_2pi
from .navigation.carrot import Carrot


def ramp(x: float, in_min: float, in_max: float, out_min: float, out_max: float, clip: bool = False):

    if clip and x < in_min:
        return out_min
    if clip and x > in_max:
        return out_max
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


async def drive_spline(spline: Spline, world: World, esp: Esp, *, throttle_at_end: bool = True):

    if spline.start.distance(spline.end) < 0.01:
        return  # NOTE: skip tiny splines

    carrot = Carrot(spline=spline)

    is_offset = world.robot.shape.point_of_interest.distance(Point(x=0, y=0)) > 0

    while True:

        point_of_interest = world.robot.prediction.transform(world.robot.shape.point_of_interest)
        if not carrot.move(point_of_interest, distance=world.robot.parameters.carrot_distance):
            break
        world.carrot = carrot.pose

        if is_offset:
            if world.robot.parameters.minimum_turning_radius:
                raise NotImplementedError('Curvature restriction is only supported for centered points-of-interest')
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
            try:
                while True:
                    local_spline = Spline.from_poses(world.robot.prediction, carrot.pose)
                    if world.robot.parameters.minimum_turning_radius:
                        break
                    max_curvature = np.abs(local_spline.max_curvature())
                    if max_curvature == 0 or 1 / max_curvature >= world.robot.parameters.minimum_turning_radius:
                        break
                    if not carrot.move(carrot.pose.point, distance=0.1):
                        raise StopIteration()
                    world.carrot = carrot.pose
            except StopIteration:
                break
            curvature = local_spline.max_curvature(0.0, 0.25)
            backward = False

        linear = 0.5
        linear *= -1 if backward else 1
        if carrot.t > 1.0 and throttle_at_end:
            linear *= ramp(carrot.target_distance, 0.5, 0.0, 1.0, 0.5)
        angular = linear * curvature

        await esp.drive(*throttle(world, linear, angular))

    world.carrot = None
    await esp.drive(0, 0)


def throttle(world: World, linear: float, angular: float):

    if world.mode != Mode.TEST:  # TODO: require camera tracking in tests as well
        if world.robot.parameters.max_detection_age_ramp is None:
            factor = 1
        elif world.robot.detection is None:
            factor = 0
        else:
            age_ramp = world.robot.parameters.max_detection_age_ramp
            age = world.time - world.robot.detection.time
            factor = ramp(age, age_ramp[0], age_ramp[1], 1.0, 0.0, clip=True)
        linear *= factor
        angular *= factor

    linear_limit = world.robot.parameters.linear_speed_limit
    angular_limit = world.robot.parameters.angular_speed_limit

    if abs(linear) > linear_limit:
        factor = linear_limit / abs(linear)
        angular *= factor
        linear *= factor
    if abs(angular) > angular_limit:
        factor = angular_limit / abs(angular)
        linear *= factor
        angular *= factor

    return linear, angular
