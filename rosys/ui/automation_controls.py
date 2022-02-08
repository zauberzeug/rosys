from nicegui.ui import Ui
from .. import Runtime


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self) -> None:

        async def start():
            await self.runtime.automator.start()

        async def pause():
            await self.runtime.automator.pause(because='pause button was pressed')

        def resume():
            self.runtime.automator.resume()

        async def stop():
            await self.runtime.automator.stop(because='stop button was pressed')

        play_button = self.ui.button(on_click=start).props('icon=play_arrow outline')
        pause_button = self.ui.button(on_click=pause).props('icon=pause outline')
        resume_button = self.ui.button(on_click=resume).props('icon=play_arrow outline')
        stop_button = self.ui.button(on_click=stop).props('icon=stop outline')

        def refresh():
            play_button.visible = self.runtime.automator.is_stopped
            pause_button.visible = self.runtime.automator.is_running
            resume_button.visible = self.runtime.automator.is_paused
            stop_button.visible = not self.runtime.automator.is_stopped
            play_button.view.disable = self.runtime.automator.default_automation is None
        self.ui.timer(0.1, refresh)
