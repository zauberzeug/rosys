from pydantic import BaseModel
from .pose import Pose
from .velocity import Velocity


class RobotShape(BaseModel):

    outline: list[tuple[float, float]] = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
    height: float = 0.5


class Robot(BaseModel):

    shape: RobotShape = RobotShape()
    prediction: Pose = Pose()
    detection: Pose = Pose()
    velocity: Velocity = Velocity()
    battery: float = 0
    temperature: float = 0
