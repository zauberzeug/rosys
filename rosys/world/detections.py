from __future__ import annotations

from dataclasses import dataclass, field

from pydantic import BaseModel

from .point import Point


class Detection(BaseModel):
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


@dataclass
class Point:
    x: float
    y: float

    def __str__(self) -> str:
        return f'x:{self.x:.0f} y: {self.y:.0f}'


@dataclass
class Shape:
    points: list[Point]

    @staticmethod
    def from_str(points: str) -> Shape:
        p = points.split(',')
        points = [Point(int(p[i]), int(p[i+1])) for i in range(0, len(p), 2)]
        return Shape(points=points)

    def __str__(self) -> str:
        return ', '. join([p.__str__ for p in self.points])


@dataclass
class SegmentationDetection:
    category_name: str
    model_name: str
    confidence: float
    shape: Shape

    @staticmethod
    def from_dict(detection: dict) -> SegmentationDetection:
        return SegmentationDetection(
            detection['category_name'],
            detection['model_name'],
            detection['confidence'],
            Shape.from_str(detection['shape']))

    def __str__(self) -> str:
        return f'shape: {self.shape.__str__} cat: {self.category_name} conf: {self.confidence:.2f}'

    def to_svg(self, shrink: int = 1) -> str:
        color = 'green'
        d = ' '.join([f"{'L' if i > 0 else 'M'} {point.x} {point.y}" for i, point in enumerate(self.shape.points)])
        d += ' Z'
        first_point = self.shape.points[0]
        return f'<path d="{d}" stroke-width="2" stroke="{color}" fill="none" />' \
            f'<text x="{first_point.x + 10}" y="{first_point.y + 4}" text-anchor="start" stroke="{color}" fill="{color}" font-size="12" font-weight="light">{self.category_name} ({int(self.confidence*100)}%)</text>'


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


@dataclass
class Detections:
    boxes: list[BoxDetection] = field(default_factory=list)
    points: list[PointDetection] = field(default_factory=list)
    segmentations: list[SegmentationDetection] = field(default_factory=list)

    def to_svg(self, shrink: int = 1) -> str:
        return '\n'.join(
            b.to_svg(shrink) for b in self.boxes) + '\n' + '\n'.join(
            p.to_svg(shrink) for p in self.points) + '\n' + '\n'.join(
            s.to_svg(shrink) for s in self.segmentations)
