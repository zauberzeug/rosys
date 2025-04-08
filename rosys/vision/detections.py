from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Literal

import numpy as np

from ..geometry import Point


@dataclass(slots=True, kw_only=True)
class Category:
    uuid: str
    name: str
    color: str | None = None
    category_type: Literal['box', 'point', 'segmentation', 'classification'] | None = None


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
        return f'x:{self.x:.0f} y:{self.y:.0f}, w:{self.width:.0f} h:{self.height:.0f} cat:{self.category_name} conf:{self.confidence:.2f}'

    def to_svg(self, shrink: int = 1, *, color: str = 'red') -> str:
        x = self.x / shrink
        y = self.y / shrink
        return (
            f'<rect x="{x}" y="{y}" width="{self.width/shrink}" height="{self.height/shrink}" stroke-width="2" '
            f'stroke="{color}" fill="none" />'
            f'<text x="{x}" y="{y - 7}" text-anchor="start" stroke="{color}" fill="{color}" font-size="10">'
            f'{self.category_name} ({self.confidence:.0%})</text>'
        )

    def intersection_over_union(self, other_detection: BoxDetection) -> float:
        # https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
        xA = max(self.x, other_detection.x)
        yA = max(self.y, other_detection.y)
        xB = min(self.x + self.width, other_detection.x + other_detection.width)
        yB = min(self.y + self.height, other_detection.y + other_detection.height)
        inter_area = max(xB - xA, 0) * max(yB - yA, 0)
        union = self.area + other_detection.area - inter_area
        if union == 0:
            print('WARNING: Something went wrong while calculating the intersection over union. Returning 0.')
            return 0
        return inter_area / union

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass(slots=True, kw_only=True)
class Shape:
    points: list[Point]

    @staticmethod
    def from_str(points: str) -> Shape:
        p = points.split(',')
        return Shape(points=[Point(x=float(p[i]), y=float(p[i+1])) for i in range(0, len(p), 2)])

    def __str__(self) -> str:
        return ', '. join([f'x:{p.x:.0f} y:{p.y:.0f}' for p in self.points])


@dataclass(slots=True, kw_only=True)
class SegmentationDetection:
    category_name: str
    model_name: str
    confidence: float
    shape: Shape

    def __str__(self) -> str:
        return f'shape:{self.shape.__str__} cat:{self.category_name} conf:{self.confidence:.2f}'

    def to_svg(self, shrink: int = 1, *, color: str = 'green') -> str:
        d = ' '.join([f"{'L' if i > 0 else 'M'} {p.x/shrink} {p.y/shrink}" for i, p in enumerate(self.shape.points)])
        d += ' Z'
        p0 = self.shape.points[0]
        return (
            f'<path d="{d}" stroke-width="2" stroke="{color}" fill="none" />'
            f'<text x="{p0.x / shrink + 10}" y="{p0.y / shrink + 4}" text-anchor="start" stroke="{color}" '
            f'fill="{color}" font-size="12" font-weight="light">{self.category_name} ({self.confidence:.0%})</text>'
        )


@dataclass(slots=True, kw_only=True)
class PointDetection(Detection):

    @property
    def cx(self) -> float:
        return self.x

    @property
    def cy(self) -> float:
        return self.y

    def distance(self, other: PointDetection) -> float:
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def __str__(self) -> str:
        return f'x:{self.x:.0f} y:{self.y:.0f} cat:{self.category_name} conf:{self.confidence:.2f}'

    def to_svg(self, shrink: int = 1, *, color: str = 'red') -> str:
        x = self.x / shrink
        y = self.y / shrink
        return (
            f'<circle cx="{x}" cy="{y}" r="4" stroke-width="2" stroke="{color}" fill="none" />'
            f'<text x="{x + 10}" y="{y + 4}" text-anchor="start" stroke="{color}" fill="{color}" font-size="12" '
            f'font-weight="light">{self.category_name} ({self.confidence:.0%})</text>'
        )


@dataclass(slots=True, kw_only=True)
class ClassificationDetection:
    category_name: str
    model_name: str
    confidence: float
    uuid: str = ''  # NOTE this is used to keep track of simulated objects

    def __str__(self) -> str:
        return f'cat:{self.category_name} conf:{self.confidence:.2f}'

    def to_svg(self) -> str:
        return (
            f'<text x="50%" y="50%" text-anchor="middle" dominant-baseline="middle" '
            f'stroke="black" fill="black" font-size="10%" font-weight="bold">'
            f'{self.category_name} ({self.confidence:.0%})</text>'
        )


@dataclass(slots=True, kw_only=True)
class Detections:
    boxes: list[BoxDetection] = field(default_factory=list)
    points: list[PointDetection] = field(default_factory=list)
    segmentations: list[SegmentationDetection] = field(default_factory=list)
    classifications: list[ClassificationDetection] = field(default_factory=list)

    def to_svg(self, shrink: int = 1) -> str:
        return '\n'.join(
            b.to_svg(shrink) for b in self.boxes) + '\n' + '\n'.join(
            p.to_svg(shrink) for p in self.points) + '\n' + '\n'.join(
            s.to_svg(shrink) for s in self.segmentations) + '\n' + '\n'.join(
            c.to_svg() for c in self.classifications)

    def to_dict(self) -> dict:
        return asdict(self)
