from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class Prism:
    outline: list[tuple[float, float]]
    height: float

    @staticmethod
    def default_robot_shape() -> Prism:
        return Prism(outline=[(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)], height=0.5)
