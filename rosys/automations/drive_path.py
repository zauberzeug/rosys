import numpy as np

from ..hardware import Hardware
from ..helpers import eliminate_2pi, sleep
from ..world import PathSegment, Point, Spline, World
from .spline import drive_spline, throttle


async def drive_path(world: World, hardware: Hardware, path: list[PathSegment]):
    for segment in path:
        await drive_spline(segment.spline, world, hardware, throttle_at_end=segment == path[-1], flip_hook=segment.backward)


async def drive_to(world: World, hardware: Hardware, target: Point):
    if world.robot.parameters.minimum_turning_radius:
        await drive_circle(world, hardware, target)

    robot_pose = world.robot.prediction
    approach_spline = Spline(
        start=robot_pose.point,
        control1=robot_pose.point.interpolate(target, 1/3),
        control2=robot_pose.point.interpolate(target, 2/3),
        end=target,
    )
    await drive_spline(approach_spline, world, hardware)


async def drive_circle(world: World, hardware: Hardware, target: Point):
    while True:
        direction = world.robot.prediction.point.direction(target)
        angle = eliminate_2pi(direction - world.robot.prediction.yaw)
        if abs(angle) < np.deg2rad(5):
            break
        linear = 0.5
        sign = 1 if angle > 0 else -1
        angular = linear / world.robot.parameters.minimum_turning_radius * sign
        await hardware.drive(*throttle(world, linear, angular))
        await sleep(0.1)
