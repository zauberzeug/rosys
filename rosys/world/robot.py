import os
from dataclasses import dataclass, field
from typing import Optional


@dataclass(slots=True, kw_only=True)
class RobotShape:
    outline: list[tuple[float, float]] = field(default_factory=lambda: [
        (-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5),
    ])
    height: float = 0.5


@dataclass(slots=True, kw_only=True)
class Robot:
    name: Optional[str] = os.environ.get('ROBOT_ID')
    shape: RobotShape = RobotShape()
    emergency_stop: bool = False
