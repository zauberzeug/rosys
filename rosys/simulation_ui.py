from datetime import timedelta

from nicegui import ui

from . import rosys


class simulation_ui:

    def __init__(self) -> None:
        ui.label('Simulation speed')
        ui.slider(min=0, max=10, value=1, step=0.1, on_change=rosys.config.request_backup) \
            .bind_value(rosys.config, 'simulation_speed').props('label-always')
        self.simulation_time = ui.label()
        self.startup_time = rosys.time()
        ui.timer(0.1, self.update_simulation_time)

    def update_simulation_time(self) -> None:
        self.simulation_time.set_text(f'Running for {timedelta(seconds=int(rosys.time() - self.startup_time))}')
