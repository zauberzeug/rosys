from pydantic import BaseModel
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


class Robot(BaseModel):

    shape: RobotShape = RobotShape()
    parameters: RobotParameters = RobotParameters()
    prediction: Pose = Pose()
    detection: Pose = Pose()
    velocity: Velocity = Velocity()
    battery: float = 0
    temperature: float = 0
