from pydantic import BaseModel


class Point3d(BaseModel):
    x: float
    y: float
    z: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)
