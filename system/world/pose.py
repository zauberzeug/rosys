from pydantic import BaseModel


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: float = 0
