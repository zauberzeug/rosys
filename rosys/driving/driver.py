from copy import deepcopy
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import rosys

from ..analysis import track
from ..geometry import Point, Pose, Spline
from ..helpers import ModificationContext, eliminate_2pi, eliminate_pi, ramp
from .drivable import Drivable
from .odometer import Odometer
from .path_segment import PathSegment


@dataclass(slots=True, kw_only=True)
class DriveParameters(ModificationContext):
    linear_speed_limit: float = 0.5
    angular_speed_limit: float = 0.5
    minimum_turning_radius: float = 0.0
    can_drive_backwards: bool = True
    max_detection_age_ramp: Optional[tuple[float, float]] = None
    hook_offset: float = 0.5
    carrot_offset: float = 0.6
    carrot_distance: float = 0.1
    hook_bending_factor: float = 0


@dataclass(slots=True, kw_only=True)
class DriveState:
    carrot_pose: Pose
    curvature: float
    backward: bool
    turn_angle: float


class Driver:
    '''The driver module allows following a given path.

    It requires a wheels module (or any drivable hardware representation) to execute individual drive commands.
    It also requires an odometer to get a current prediction of the robot's pose.
    Its `parameters` allow controlling the specific drive behavior.
    '''

    def __init__(self, wheels: Drivable, odometer: Odometer) -> None:
        self.wheels = wheels
        self.odometer = odometer
        self.parameters = DriveParameters()
        self.state: Optional[DriveState] = None

    @track
    async def drive_square(self) -> None:
        start_pose = deepcopy(self.odometer.prediction)
        for x, y in [(1, 0), (1, 1), (0, 1), (0, 0)]:
            await self.drive_to(start_pose.transform(Point(x=x, y=y)))

    @track
    async def drive_arc(self) -> None:
        while self.odometer.prediction.x < 2:
            await self.wheels.drive(1, np.deg2rad(25))
            await rosys.sleep(0.1)
        await self.wheels.stop()

    @track
    async def drive_path(self, path: list[PathSegment]) -> None:
        for segment in path:
            await self.drive_spline(segment.spline, throttle_at_end=segment == path[-1], flip_hook=segment.backward)

    @track
    async def drive_to(self, target: Point) -> None:
        if self.parameters.minimum_turning_radius:
            await self.drive_circle(target)

        robot_position = self.odometer.prediction.point
        approach_spline = Spline(
            start=robot_position,
            control1=robot_position.interpolate(target, 1/3),
            control2=robot_position.interpolate(target, 2/3),
            end=target,
        )
        await self.drive_spline(approach_spline)

    @track
    async def drive_circle(self, target: Point) -> None:
        while True:
            direction = self.odometer.prediction.point.direction(target)
            angle = eliminate_2pi(direction - self.odometer.prediction.yaw)
            if abs(angle) < np.deg2rad(5):
                break
            linear = 0.5
            sign = 1 if angle > 0 else -1
            angular = linear / self.parameters.minimum_turning_radius * sign
            await self.wheels.drive(*self._throttle(linear, angular))
            await rosys.sleep(0.1)

    @track
    async def drive_spline(self, spline: Spline, *, flip_hook: bool = False, throttle_at_end: bool = True) -> None:
        if spline.start.distance(spline.end) < 0.01:
            return  # NOTE: skip tiny splines

        hook_offset = Point(x=self.parameters.hook_offset, y=0) * (-1 if flip_hook else 1)
        carrot_offset = Point(x=self.parameters.carrot_offset, y=0)
        carrot = Carrot(spline=spline, offset=carrot_offset)

        while True:
            dYaw = self.parameters.hook_bending_factor * self.odometer.current_velocity.angular if self.odometer.current_velocity else 0
            hook = self.odometer.prediction.transform_pose(Pose(yaw=dYaw)).transform(hook_offset)
            if self.parameters.can_drive_backwards:
                can_move = carrot.move(hook, distance=self.parameters.carrot_distance)
            else:
                can_move = carrot.move_by_foot(self.odometer.prediction)
            if not can_move:
                break

            turn_angle = eliminate_pi(hook.direction(carrot.offset_point) - self.odometer.prediction.yaw)
            curvature = np.tan(turn_angle) / hook_offset.x
            if curvature != 0 and abs(1 / curvature) < self.parameters.minimum_turning_radius:
                curvature = (-1 if curvature < 0 else 1) / self.parameters.minimum_turning_radius

            drive_backward = hook.projected_distance(carrot.offset_point, self.odometer.prediction.yaw) < 0
            if drive_backward and not self.parameters.can_drive_backwards:
                drive_backward = False
                curvature = (-1 if curvature > 0 else 1) / max(self.parameters.minimum_turning_radius, 0.001)
            linear = -1 if drive_backward else 1
            if carrot.t > 1.0 and throttle_at_end:
                linear *= ramp(carrot.target_distance, 0.5, 0.0, 1.0, 0.5)
            angular = linear * curvature

            self.state = DriveState(
                carrot_pose=carrot.pose,
                curvature=curvature,
                backward=drive_backward,
                turn_angle=turn_angle,
            )

            await self.wheels.drive(*self._throttle(linear, angular))
            await rosys.sleep(0.1)

        self.state = None
        await self.wheels.stop()

    def _throttle(self, linear: float, angular: float) -> tuple[float, float]:
        factor = self.throttle_factor()
        linear *= factor
        angular *= factor

        linear_limit = self.parameters.linear_speed_limit
        angular_limit = self.parameters.angular_speed_limit

        if abs(linear) > linear_limit:
            factor = linear_limit / abs(linear)
            angular *= factor
            linear *= factor
        if abs(angular) > angular_limit:
            factor = angular_limit / abs(angular)
            linear *= factor
            angular *= factor

        return linear, angular

    def throttle_factor(self) -> float:
        if self.parameters.max_detection_age_ramp is None:
            return 1
        elif self.odometer.detection is None:
            return 0
        else:
            age_ramp = self.parameters.max_detection_age_ramp
            age = rosys.time() - self.odometer.detection.time
            return ramp(age, age_ramp[0], age_ramp[1], 1.0, 0.0, clip=True)


@dataclass(slots=True, kw_only=True)
class Carrot:
    spline: Spline
    offset: Point = field(default_factory=lambda: Point(x=0, y=0))
    t: float = 0
    target_distance: Optional[float] = None

    @property
    def pose(self) -> Pose:
        if self.t < 1.0:
            return self.spline.pose(self.t)
        else:
            return Pose(
                x=self.spline.x(1.0) + (self.t - 1.0) * self.spline.gx(1.0),
                y=self.spline.y(1.0) + (self.t - 1.0) * self.spline.gy(1.0),
                yaw=self.spline.yaw(1.0),
            )

    @property
    def offset_point(self) -> Point:
        return self.pose.transform(self.offset)

    def move(self, hook: Point, distance: float = 1.0, move_threshold: float = 0.01) -> bool:
        end_pose = self.spline.pose(1.0)
        end_point = end_pose.point
        end_yaw = end_pose.yaw

        while hook.distance(self.offset_point) < distance:
            self.t += 0.01
            if self.t < 1.0:
                continue
            self.target_distance = hook.projected_distance(end_point, end_yaw)
            if self.target_distance <= move_threshold:
                return False

        return True

    def move_by_foot(self, pose: Pose) -> bool:
        self.t = self.spline.closest_point(pose.x, pose.y, t_min=self.t, t_max=1.0)
        return self.t < 1.0
