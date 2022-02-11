from typing import Awaitable, Optional
from nicegui.ui import Ui
from .. import Runtime


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self, default_automation: Optional[Awaitable] = None) -> None:

        def start():
            self.runtime.automator.start(default_automation())

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
            play_button.visible = self.runtime.automator.is_stopped
            pause_button.visible = self.runtime.automator.is_running
            resume_button.visible = self.runtime.automator.is_paused
            play_button.view.disable = default_automation is None
            stop_button.view.disable = self.runtime.automator.is_stopped
        self.ui.timer(0.1, refresh)
