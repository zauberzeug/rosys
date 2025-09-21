from dataclasses import dataclass, field
from typing import Any, Protocol, Self

import numpy as np
from nicegui import binding, ui

from .. import rosys
from ..analysis import track
from ..geometry import Point, Pose, Spline
from ..helpers import ModificationContext, eliminate_2pi, eliminate_pi, ramp
from ..persistence import Persistable
from .drivable import Drivable
from .odometer import Odometer
from .path_segment import PathSegment


# slots=True, kw_only=True
@binding.bindable_dataclass
class DriveParameters(ModificationContext):
    linear_speed_limit: float = 0.5
    angular_speed_limit: float = 0.5
    minimum_turning_radius: float = 0.0
    can_drive_backwards: bool = True
    max_detection_age_ramp: tuple[float, float] | None = None
    hook_offset: float = 0.5
    carrot_offset: float = 0.6
    carrot_distance: float = 0.1
    hook_bending_factor: float = 0
    minimum_drive_distance: float = 0.01
    throttle_at_end_distance: float = 0.5
    throttle_at_end_min_speed: float = 0.01

    def to_dict(self) -> dict[str, Any]:
        return {
            'linear_speed_limit': self.linear_speed_limit,
            'angular_speed_limit': self.angular_speed_limit,
            'minimum_turning_radius': self.minimum_turning_radius,
            # TODO: Backup failed: Object of type bool_ is not JSON serializable from /Users/pascal/.rosys/field_friend.driver.json
            'can_drive_backwards': bool(self.can_drive_backwards),
            'max_detection_age_ramp': self.max_detection_age_ramp,
            'hook_offset': self.hook_offset,
            'carrot_offset': self.carrot_offset,
            'carrot_distance': self.carrot_distance,
            'hook_bending_factor': self.hook_bending_factor,
            'minimum_drive_distance': self.minimum_drive_distance,
            'throttle_at_end_distance': self.throttle_at_end_distance,
            'throttle_at_end_min_speed': self.throttle_at_end_min_speed,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**data)


@dataclass(slots=True, kw_only=True)
class DriveState:
    carrot_pose: Pose
    curvature: float
    backward: bool
    turn_angle: float


class DrivingAbortedException(Exception):
    pass


class PoseProvider(Protocol):

    @property
    def pose(self) -> Pose:
        ...


class Driver(Persistable):
    """The driver module allows following a given path.

    It requires a wheels module (or any drivable hardware representation) to execute individual drive commands.
    It also requires an odometer to get a current prediction of the robot's pose.
    Its `parameters` allow controlling the specific drive behavior.
    """

    def __init__(self, wheels: Drivable, odometer: Odometer | PoseProvider, *, parameters: DriveParameters | None = None) -> None:
        super().__init__()
        self.wheels = wheels
        self.odometer = odometer
        self.parameters = parameters or DriveParameters()
        self.state: DriveState | None = None
        self._abort = False

    @property
    def prediction(self) -> Pose:
        """The current prediction of the robot's pose based on the odometer."""
        return self.odometer.prediction if isinstance(self.odometer, Odometer) else self.odometer.pose

    def abort(self) -> None:
        """Abort the current drive routine."""
        self._abort = True

    @track
    async def drive_path(self,
                         path: list[PathSegment], *,
                         throttle_at_end: bool = True,
                         stop_at_end: bool = True) -> None:
        """Drive along a given path.

        :param path: The path to drive along, composed of PathSegments.
        :param throttle_at_end: Whether to throttle down when approaching the end of the path (default: ``True``).
        :param stop_at_end: Whether to stop at the end of the path (default: ``True``).
        :raises: DrivingAbortedException: If the driving process is aborted.
        """
        for segment in path:
            await self.drive_spline(segment.spline,
                                    flip_hook=segment.backward,
                                    throttle_at_end=throttle_at_end and segment == path[-1],
                                    stop_at_end=stop_at_end and segment == path[-1])

    @track
    async def drive_to(self,
                       target: Point, *,
                       backward: bool = False,
                       throttle_at_end: bool = True,
                       stop_at_end: bool = True) -> None:
        """Drive to a given target point.

        :param target: The target point to drive to.
        :param backward: Whether to drive backwards (default: ``False``).
        :param throttle_at_end: Whether to throttle down when approaching the target point (default: ``True``).
        :param stop_at_end: Whether to stop at the target point (default: ``True``).
        :raises: DrivingAbortedException: If the driving process is aborted.
        """
        if self.parameters.minimum_turning_radius:
            await self.drive_circle(target, backward=backward, stop_at_end=False)

        robot_position = self.prediction.point
        approach_spline = Spline(
            start=robot_position,
            control1=robot_position.interpolate(target, 1/3),
            control2=robot_position.interpolate(target, 2/3),
            end=target,
        )
        await self.drive_spline(approach_spline, flip_hook=backward, throttle_at_end=throttle_at_end, stop_at_end=stop_at_end)

    @track
    async def drive_circle(self,
                           target: Point, *,
                           angle_threshold: float = np.deg2rad(5),
                           backward: bool = False,
                           stop_at_end: bool = True) -> None:
        """Drive in a circular path.

        When the angle between the robot's current direction and the target direction is less than ``angle_threshold``,
        the robot stops driving.

        :param target: The target point to drive towards.
        :param angle_threshold: The angle threshold to stop driving (radians, default: 5°).
        :param backward: Whether to drive backwards (default: ``False``).
        :param stop_at_end: Whether to stop the robot at the end of the circular path (default: ``False``).
        :raises: DrivingAbortedException: If the driving process is aborted.
        """
        while True:
            if self._abort:
                self._abort = False
                raise DrivingAbortedException()
            target_yaw = self.prediction.direction(target)
            if backward:
                target_yaw += np.pi
            angle = eliminate_2pi(target_yaw - self.prediction.yaw)
            if abs(angle) < angle_threshold:
                break
            linear = 0.5 if not backward else -0.5
            sign = 1 if angle > 0 else -1
            if backward:
                sign *= -1
            angular = linear / self.parameters.minimum_turning_radius * sign
            await self.wheels.drive(*self._throttle(linear, angular))
            await rosys.sleep(0.1)
        if stop_at_end:
            await self.wheels.stop()

    @track
    async def drive_spline(self,
                           spline: Spline, *,
                           flip_hook: bool = False,
                           throttle_at_end: bool = True,
                           stop_at_end: bool = True) -> None:
        """Drive along a given spline.

        :param spline: The spline to drive along.
        :param flip_hook: Whether to flip the hook offset (default: ``False``).
        :param throttle_at_end: Whether to throttle down when approaching the end of the spline (default: ``True``).
        :param stop_at_end: Whether to stop at the end of the spline (default: ``True``).
        :raises DrivingAbortedException: If the driving process is aborted.
        """
        if spline.start.distance(spline.end) < self.parameters.minimum_drive_distance:
            if stop_at_end:
                await self.wheels.stop()
            return  # NOTE: skip tiny splines

        hook_offset = Point(x=self.parameters.hook_offset, y=0) * (-1 if flip_hook else 1)
        carrot_offset = Point(x=self.parameters.carrot_offset, y=0)
        carrot = Carrot(spline=spline, offset=carrot_offset)

        while True:
            if self._abort:
                self._abort = False
                raise DrivingAbortedException()
            velocity = self.odometer.current_velocity if isinstance(self.odometer, Odometer) else None
            dYaw = self.parameters.hook_bending_factor * velocity.angular if velocity else 0
            hook = self.prediction.transform_pose(Pose(yaw=dYaw)).transform(hook_offset)
            if self.parameters.can_drive_backwards:
                can_move = carrot.move(hook, distance=self.parameters.carrot_distance)
            else:
                can_move = carrot.move_by_foot(self.prediction)
            if not can_move:
                break

            turn_angle = eliminate_pi(hook.direction(carrot.offset_point) - self.prediction.yaw)
            curvature = np.tan(turn_angle) / hook_offset.x
            if curvature != 0 and abs(1 / curvature) < self.parameters.minimum_turning_radius:
                curvature = (-1 if curvature < 0 else 1) / self.parameters.minimum_turning_radius

            drive_backward = hook.projected_distance(carrot.offset_point, self.prediction.yaw) < 0
            if drive_backward and not self.parameters.can_drive_backwards:
                drive_backward = False
                curvature = (-1 if curvature > 0 else 1) / max(self.parameters.minimum_turning_radius, 0.001)
            linear: float = self.parameters.linear_speed_limit * (-1 if drive_backward else 1)
            t = spline.closest_point(hook.x, hook.y)
            if t >= 1.0 and throttle_at_end:
                target_distance = self.prediction.projected_distance(spline.pose(1.0))
                throttle_distance = self.parameters.throttle_at_end_distance
                min_linear_speed = self.parameters.throttle_at_end_min_speed
                linear *= ramp(target_distance, throttle_distance, 0.0, 1.0, min_linear_speed, clip=True)
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
        if stop_at_end:
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
        if not isinstance(self.odometer, Odometer):
            raise ValueError('max_detection_age_ramp requires an odometer')
        if self.odometer.detection is None:
            return 0
        age_ramp = self.parameters.max_detection_age_ramp
        age = rosys.time() - self.odometer.detection.time
        return ramp(age, age_ramp[0], age_ramp[1], 1.0, 0.0, clip=True)

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'parameters': self.parameters.to_dict(),
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.parameters = DriveParameters.from_dict(data['parameters'])

    def developer_ui(self, *, columns: int = 2) -> None:
        ui.label('Driver').classes('text-center text-bold')
        with ui.grid(columns=columns):
            for name, value in self.parameters.to_dict().items():
                if isinstance(value, float):
                    ui.number(name, value=value, on_change=self.request_backup).bind_value_to(self.parameters, name)
                elif isinstance(value, bool):
                    ui.checkbox(name, value=value, on_change=self.request_backup).bind_value_to(self.parameters, name)
                elif name == 'max_detection_age_ramp':
                    self.log.warning('max_detection_age_ramp is not supported in the UI')
                else:
                    self.log.error('Unknown type %s of %s', type(value), name)


@dataclass(slots=True, kw_only=True)
class Carrot:
    spline: Spline
    offset: Point = field(default_factory=lambda: Point(x=0, y=0))
    t: float = 0
    _estimated_spline_length: float = field(init=False)

    def __post_init__(self) -> None:
        self._estimated_spline_length = self.spline.estimated_length()  # NOTE: compute once to avoid repeated computation

    @property
    def pose(self) -> Pose:
        if self.t < 1.0:
            return self.spline.pose(self.t)
        return Pose(
            x=self.spline.x(1.0) + (self.t - 1.0) * self.spline.gx(1.0),
            y=self.spline.y(1.0) + (self.t - 1.0) * self.spline.gy(1.0),
            yaw=self.spline.yaw(1.0),
        )

    @property
    def offset_point(self) -> Point:
        return self.pose.transform(self.offset)

    def move(self, hook: Point, distance: float) -> bool:
        dt = 0.1 * distance / self._estimated_spline_length
        while hook.distance(self.offset_point) < distance:
            self.t += dt
            if self.t >= 1.0:
                return False
        return True

    def move_by_foot(self, pose: Pose) -> bool:
        self.t = self.spline.closest_point(pose.x, pose.y, t_min=self.t, t_max=1.0)
        return self.t < 1.0
