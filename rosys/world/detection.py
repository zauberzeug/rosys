from pydantic import BaseModel


class Detection(BaseModel):
    category_name: str
    model_name: str
    confidence: float
    x: float
    y: float

    @property
    def cx(self) -> float:
        raise NotImplementedError()

    @property
    def cy(self) -> float:
        raise NotImplementedError()


class BoxDetection(Detection):

    width: float
    height: float

    @property
    def cx(self) -> float:
        return self.x + self.width / 2

    @property
    def cy(self) -> float:
        return self.y + self.height / 2

    def __str__(self):
        return f'x:{self.x:.0f} y: {self.y:.0f}, w: {self.width:.0f} h: {self.height:.0f} cat: {self.category_name} conf: {self.confidence:.2f}'


class PointDetection(Detection):

    @property
    def cx(self) -> float:
        return self.x

    @property
    def cy(self) -> float:
        return self.y

    def __str__(self):
        return f'x:{self.x:.0f} y: {self.y:.0f} cat: {self.category_name} conf: {self.confidence:.2f}'
