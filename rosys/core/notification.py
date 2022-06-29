from dataclasses import dataclass


@dataclass
class Notification:
    time: float
    message: str
