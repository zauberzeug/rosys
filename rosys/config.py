from dataclasses import dataclass
from typing import Any

from . import rosys


@dataclass(slots=True, kw_only=True)
class Config:
    ui_update_interval: float = 0.1
    simulation_speed: float = 1.0
    needs_backup: bool = False

    def __post_init__(self) -> None:
        rosys.persistence.register(self)

    def backup(self) -> dict[str, Any]:
        return {'simulation_speed': self.simulation_speed}

    def restore(self, data: dict[str, Any]) -> None:
        self.simulation_speed = data.get('simulation_speed', 1.0)
