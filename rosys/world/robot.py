import os
from typing import Optional

from pydantic import BaseModel

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
    emergency_stop: bool = False
    carrot: Optional[Pose]
