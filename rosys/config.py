from typing import Any

from . import persistence


class Config(persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        self.ui_update_interval: float = 0.1
        self.simulation_speed: float = 1.0

    def backup(self) -> dict[str, Any]:
        return {'simulation_speed': self.simulation_speed}

    def restore(self, data: dict[str, Any]) -> None:
        self.simulation_speed = data.get('simulation_speed', 1.0)
