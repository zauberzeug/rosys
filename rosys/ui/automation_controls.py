import inspect
from typing import Awaitable, Callable, Optional, Union

from nicegui.ui import Ui

from .. import Runtime
from .settings import update_interval


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self,
                 default_automation: Optional[Awaitable] = None,
                 can_start: Optional[Callable[[], Union[bool, Awaitable[bool]]]] = None) -> None:

        async def start():
            if inspect.iscoroutinefunction(can_start):
                if not await can_start():
                    return
            elif can_start is not None and not can_start():
                return
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
        self.ui.timer(update_interval, refresh)
