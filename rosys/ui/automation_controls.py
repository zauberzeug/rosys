from nicegui.ui import Ui
from .. import Runtime
from ..world import AutomationState


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self) -> None:

        def start():
            self.runtime.automator.start()

        def pause():
            self.runtime.automator.pause(because='pause button was pressed')

        def resume():
            self.runtime.automator.resume()

        def stop():
            self.runtime.automator.stop(because='stop button was pressed')

        play_button = self.ui.button(on_click=start).props('icon=play_arrow outline')
        pause_button = self.ui.button(on_click=pause).props('icon=pause outline')
        resume_button = self.ui.button(on_click=resume).props('icon=play_arrow outline')
        stop_button = self.ui.button(on_click=stop).props('icon=stop outline')

        def refresh():
            state = self.runtime.world.automation_state
            play_button.visible = state == AutomationState.STOPPED
            pause_button.visible = state == AutomationState.RUNNING
            resume_button.visible = state == AutomationState.PAUSED
            stop_button.visible = state in [AutomationState.RUNNING, AutomationState.PAUSED]
            play_button.view.disable = self.runtime.automator.default_automation is None
        self.ui.timer(0.1, refresh)
