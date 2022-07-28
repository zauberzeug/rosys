from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class Velocity:
    linear: float
    angular: float
    time: float
