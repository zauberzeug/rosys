from pydantic import BaseModel


class Velocity(BaseModel):

    linear: float = 0
    angular: float = 0
