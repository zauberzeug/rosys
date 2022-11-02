from typing import Any

from nicegui import ui

import rosys


class simulation_ui:

    def __init__(self) -> None:
        ui.label('Simulation speed')
        ui.slider(min=0, max=10, value=1, step=0.1, on_change=self.request_backup).bind_value(rosys, 'speed') \
            .props('label-always').classes('mt-6')
        self.needs_backup: bool = False
        rosys.persistence.register(self)

    def request_backup(self) -> None:
        self.needs_backup = True

    def backup(self) -> dict[str, Any]:
        return {'speed': rosys.speed}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.speed = data.get('speed', 1.0)
