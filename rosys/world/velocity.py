from pydantic import BaseModel


class Velocity(BaseModel):
    linear: float
    angular: float
    time: float
