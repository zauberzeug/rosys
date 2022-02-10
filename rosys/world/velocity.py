from dataclasses import dataclass


@dataclass
class Velocity:
    linear: float
    angular: float
    time: float
