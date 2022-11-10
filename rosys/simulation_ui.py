from datetime import timedelta
from typing import Any

import humanize
from nicegui import ui

import rosys


class simulation_ui:

    def __init__(self) -> None:
        ui.label('Simulation speed')
        ui.slider(min=0, max=10, value=1, step=0.1, on_change=self.request_backup).bind_value(rosys, 'speed') \
            .props('label-always')
        self.simulation_time = ui.label()
        self.startup_time = rosys.time()
        self.needs_backup: bool = False
        rosys.persistence.register(self)
        ui.timer(0.1, self.update_simulation_time)

    def request_backup(self) -> None:
        self.needs_backup = True

    def backup(self) -> dict[str, Any]:
        return {'speed': rosys.speed}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.speed = data.get('speed', 1.0)

    def update_simulation_time(self) -> None:
        self.simulation_time.set_text(f' Running for {timedelta(seconds=int(rosys.time() -self.startup_time))}')
