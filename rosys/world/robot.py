import os
from pydantic import BaseModel
from typing import Optional
from ..helpers import ModificationContext
from .pose import Pose
from .velocity import Velocity


class RobotShape(BaseModel):
    outline: list[tuple[float, float]] = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
    height: float = 0.5


class RobotParameters(BaseModel, ModificationContext):
    linear_speed_limit: float = 0.5
    angular_speed_limit: float = 0.5
    minimum_turning_radius: float = 0.0
    max_detection_age_ramp: Optional[tuple[float, float]]
    hook_offset: float = 0.5
    carrot_offset: float = 0.6
    carrot_distance: float = 0.1


class Robot(BaseModel):
    name: Optional[str] = os.environ.get('ROBOT_ID')
    shape: RobotShape = RobotShape()
    parameters: RobotParameters = RobotParameters()
    prediction: Pose = Pose()
    detection: Optional[Pose]
    simulation: Pose = Pose()
    odometry: list[Velocity] = []
    current_velocity: Optional[Velocity]
    last_movement: float = 0
    hardware_time: Optional[float]
    battery: float = 0
    is_charging: bool = False
    temperature: float = 0
    emergency_stop: bool = False
    clock_offset: Optional[float]
    carrot: Optional[Pose]
