from nicegui.ui import Ui
from .. import Runtime
from ..world.world import AutomationState


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self) -> None:

        async def toggle_automation():
            if self.runtime.world.automation_state != AutomationState.RUNNING:
                await self.runtime.resume()
            else:
                await self.runtime.pause(because='pause button was pressed')

        async def stop():
            await self.runtime.stop(because='stop button was pressed')

        automation_button = self.ui.button(on_click=toggle_automation).props('icon=play_arrow outline')
        stop_button = self.ui.button(on_click=stop).props('icon=stop outline')

        def refresh():
            if self.runtime.world.automation_state == AutomationState.DISABLED:
                automation_button.props(replace='icon=play_arrow')
                automation_button.view.disable = stop_button.view.disable = True
            elif self.runtime.world.automation_state == AutomationState.STOPPED:
                automation_button.props(replace='icon=play_arrow')
                automation_button.view.disable = False
                stop_button.view.disable = True
            elif self.runtime.world.automation_state == AutomationState.RUNNING:
                automation_button.props(replace='icon=pause')
                automation_button.view.disable = stop_button.view.disable = False
            elif self.runtime.world.automation_state == AutomationState.PAUSED:
                automation_button.props(replace='icon=play_arrow')
                automation_button.view.disable = stop_button.view.disable = False
        self.ui.timer(0.1, refresh)
