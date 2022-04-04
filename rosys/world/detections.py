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

    def __str__(self) -> str:
        return f'x:{self.x:.0f} y: {self.y:.0f}, w: {self.width:.0f} h: {self.height:.0f} cat: {self.category_name} conf: {self.confidence:.2f}'

    def to_svg(self) -> str:
        color = 'red'
        return f'<rect x="{self.x}" y="{self.y}" width="{self.width}" height="{self.height}" stroke-width="2" stroke="{color}" fill="none" />' \
            f'<text x="{self.x}" y="{self.y - 7}" text-anchor="start" stroke="{color}" fill="{color}" font-size="10">{self.category_name} ({int(self.confidence*100)}%)</text>'


class PointDetection(Detection):

    @property
    def cx(self) -> float:
        return self.x

    @property
    def cy(self) -> float:
        return self.y

    def __str__(self) -> str:
        return f'x:{self.x:.0f} y: {self.y:.0f} cat: {self.category_name} conf: {self.confidence:.2f}'

    def to_svg(self) -> str:
        color = 'red'
        return f'<circle cx="{self.x}" cy="{self.y}" r="4" stroke-width="2" stroke="{color}" fill="none" />' \
            f'<text x="{self.x + 10}" y="{self.y+4}" text-anchor="start" stroke="{color}" fill="{color}" font-size="12" font-weight="light">{self.category_name} ({int(self.confidence*100)}%)</text>'


class Detections(BaseModel):
    boxes: list[BoxDetection] = []
    points: list[PointDetection] = []

    def to_svg(self) -> str:
        return '\n'.join(b.to_svg() for b in self.boxes) + '\n' + '\n'.join(p.to_svg() for p in self.points)
