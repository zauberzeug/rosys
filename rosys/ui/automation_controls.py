from nicegui.ui import Ui

from .. import Runtime
from .. import task_logger
from ..automations import drive_path
from ..world.world import WorldState


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self) -> None:
        def pause():
            task_logger.create_task(self.runtime.pause())
            self.runtime.world.tracking = False

        def toggle_automation():
            if self.runtime.world.state == WorldState.PAUSED:
                if not self.runtime.automator.routines:
                    self.runtime.automator.add(drive_path(self.runtime.world, self.runtime.esp))
                self.runtime.resume()
                self.runtime.world.tracking = True
            elif self.runtime.world.state == WorldState.RUNNING:
                pause()

        def stop():
            pause()
            self.runtime.automator.routines.clear()

        automation_button = self.ui.button(on_click=toggle_automation).props('icon=play_arrow outline')
        self.ui.button(on_click=stop).props('icon=stop outline')

        def refresh_steering():
            if self.runtime.world.state == WorldState.RUNNING:
                automation_button.props(replace='icon=pause')
            if self.runtime.world.state == WorldState.PAUSED:
                automation_button.props(replace='icon=play_arrow')
        self.ui.timer(0.1, refresh_steering)
