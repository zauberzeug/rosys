from typing import Any

from .persistence.persistable import Persistable


class Config(Persistable):

    def __init__(self) -> None:
        super().__init__()
        self.ui_update_interval: float = 0.1
        self.simulation_speed: float = 1.0

    def backup_to_dict(self) -> dict[str, Any]:
        return {'simulation_speed': self.simulation_speed}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.simulation_speed = data.get('simulation_speed', 1.0)
