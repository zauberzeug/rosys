from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass(slots=True, kw_only=True)
class BoxAnnotation:
    category_name: str
    x: int
    y: int
    width: int
    height: int


@dataclass(slots=True, kw_only=True)
class PointAnnotation:
    category_name: str
    x: float
    y: float


@dataclass(slots=True, kw_only=True)
class ClassificationAnnotation:
    category_name: str


@dataclass(slots=True, kw_only=True)
class Annotations:
    box_annotations: list[BoxAnnotation] = field(default_factory=list)
    point_annotations: list[PointAnnotation] = field(default_factory=list)
    classification_annotation: ClassificationAnnotation | None = None

    def to_dict(self) -> dict:
        return asdict(self)
