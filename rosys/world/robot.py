import os
from typing import Optional

from pydantic import BaseModel


class RobotShape(BaseModel):
    outline: list[tuple[float, float]] = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
    height: float = 0.5


class Robot(BaseModel):
    name: Optional[str] = os.environ.get('ROBOT_ID')
    shape: RobotShape = RobotShape()
    emergency_stop: bool = False
