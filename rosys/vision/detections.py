from dataclasses import dataclass, field

from ..geometry import Point


@dataclass(slots=True, kw_only=True)
class Detection:
    category_name: str
    model_name: str
    confidence: float
    x: float
    y: float
    uuid: str = ''  # NOTE this is used to keep track of simulated objects

    @property
    def cx(self) -> float:
        raise NotImplementedError()

    @property
    def cy(self) -> float:
        raise NotImplementedError()

    @property
    def center(self) -> Point:
        return Point(x=self.cx, y=self.cy)


@dataclass(slots=True, kw_only=True)
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

    def to_svg(self, shrink: int = 1) -> str:
        color = 'red'
        x = self.x / shrink
        y = self.y / shrink
        return f'<rect x="{x}" y="{y}" width="{self.width/shrink}" height="{self.height/shrink}" stroke-width="2" stroke="{color}" fill="none" />' \
            f'<text x="{x}" y="{y - 7}" text-anchor="start" stroke="{color}" fill="{color}" font-size="10">{self.category_name} ({int(self.confidence*100)}%)</text>'


@dataclass(slots=True, kw_only=True)
class PointDetection(Detection):

    @property
    def cx(self) -> float:
        return self.x

    @property
    def cy(self) -> float:
        return self.y

    def __str__(self) -> str:
        return f'x:{self.x:.0f} y: {self.y:.0f} cat: {self.category_name} conf: {self.confidence:.2f}'

    def to_svg(self, shrink: int = 1) -> str:
        color = 'red'
        x = self.x / shrink
        y = self.y / shrink
        return f'<circle cx="{x}" cy="{y}" r="4" stroke-width="2" stroke="{color}" fill="none" />' \
            f'<text x="{x + 10}" y="{y + 4}" text-anchor="start" stroke="{color}" fill="{color}" font-size="12" font-weight="light">{self.category_name} ({int(self.confidence*100)}%)</text>'


@dataclass(slots=True, kw_only=True)
class Detections:
    boxes: list[BoxDetection] = field(default_factory=list)
    points: list[PointDetection] = field(default_factory=list)

    def to_svg(self, shrink: int = 1) -> str:
        return '\n'.join(b.to_svg(shrink) for b in self.boxes) + '\n' + '\n'.join(p.to_svg(shrink) for p in self.points)
