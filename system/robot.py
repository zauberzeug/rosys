from pydantic import BaseModel


class Speed(BaseModel):
    linear: float = 0
    angular: float = 0


class Drive(BaseModel):
    left: float = 0
    right: float = 0


class Robot(BaseModel):
    drive: Drive = Drive(x=0, y=0)
