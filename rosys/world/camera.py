
from collections import deque
from typing import Any
from pydantic import BaseModel, Field
from dataclasses import dataclass


@dataclass
class Frame():
    time: int  # World time of recording
    data: Any


class Camera(BaseModel):
    exposure: float = 0
    capture: bool = True
    frames: deque[Frame] = Field(deque(maxlen=10), exclude=True)
