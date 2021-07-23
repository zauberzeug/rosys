from pydantic import BaseModel
from typing import Optional
from .pose import Pose
from .point import Point
from .velocity import Velocity


class RobotShape(BaseModel):

    outline: list[tuple[float, float]] = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
    height: float = 0.5
    point_of_interest: Point = Point(x=0, y=0)


class RobotParameters(BaseModel):

    linear_speed_limit: float = 0.5
    angular_speed_limit: float = 0.5
    carrot_distance: float = 1.0
    maximum_curvature: Optional[float]


class Robot(BaseModel):

    shape: RobotShape = RobotShape()
    parameters: RobotParameters = RobotParameters()
    prediction: Pose = Pose()
    detection: Pose = Pose()
    simulation: Pose = Pose()
    odometry: list[Velocity] = []
    battery: float = 0
    temperature: float = 0
    clock_offset: Optional[float]
