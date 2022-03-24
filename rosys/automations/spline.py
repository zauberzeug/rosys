import numpy as np

from ..hardware import Hardware
from ..helpers import eliminate_pi, is_test, sleep
from ..world import Point, Spline, World
from .navigation import Carrot


def ramp(x: float, in_min: float, in_max: float, out_min: float, out_max: float, clip: bool = False):
    if clip and x < in_min:
        return out_min
    if clip and x > in_max:
        return out_max
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


async def drive_spline(spline: Spline, world: World, hardware: Hardware, *, flip_hook: bool = False, throttle_at_end: bool = True):
    if spline.start.distance(spline.end) < 0.01:
        return  # NOTE: skip tiny splines

    hook_offset = Point(x=world.robot.parameters.hook_offset, y=0) * (-1 if flip_hook else 1)
    carrot_offset = Point(x=world.robot.parameters.carrot_offset, y=0)
    carrot = Carrot(spline=spline, offset=carrot_offset)

    while True:
        hook = world.robot.prediction.transform(hook_offset)
        if not carrot.move(hook, distance=world.robot.parameters.carrot_distance):
            break
        world.robot.carrot = carrot.pose

        turn_angle = eliminate_pi(hook.direction(carrot.offset_point) - world.robot.prediction.yaw)
        curvature = np.tan(turn_angle) / hook_offset.x
        if curvature != 0 and abs(1 / curvature) < world.robot.parameters.minimum_turning_radius:
            curvature = (-1 if curvature < 0 else 1) / world.robot.parameters.minimum_turning_radius

        drive_backward = hook.projected_distance(carrot.offset_point, world.robot.prediction.yaw) < 0
        linear = -1 if drive_backward else 1
        if carrot.t > 1.0 and throttle_at_end:
            linear *= ramp(carrot.target_distance, 0.5, 0.0, 1.0, 0.5)
        angular = linear * curvature

        await hardware.drive(*throttle(world, linear, angular))
        await sleep(0.1)

    world.robot.carrot = None
    await hardware.drive(0, 0)


def throttle(world: World, linear: float, angular: float):
    if not is_test:  # TODO: require camera tracking in tests as well
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
