import inspect
from typing import Awaitable, Callable, Optional, Union

from nicegui.ui import Ui
from rosys.hardware import AppButton, AppControls

from .. import Runtime
from .settings import update_interval


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self, *,
                 default_automation: Optional[Awaitable] = None,
                 can_start: Optional[Callable[[], Union[bool, Awaitable[bool]]]] = None,
                 app_conntrols: AppControls = None,
                 ) -> None:

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

        play_button = self.ui.button(on_click=start).props('icon=play_arrow unelevated')
        pause_button = self.ui.button(on_click=pause).props('icon=pause outline')
        resume_button = self.ui.button(on_click=resume).props('icon=play_arrow outline')
        stop_button = self.ui.button(on_click=stop).props('icon=stop outline')

        if app_conntrols is not None:
            app_conntrols.buttons['play'] = AppButton('play_arrow', 'enabled', released=start)
            app_conntrols.buttons['pause'] = AppButton('pause', 'enabled', released=pause)
            app_conntrols.buttons['resume'] = AppButton('play_arrow', 'enabled', released=resume)
            app_conntrols.buttons['stop'] = AppButton('stop', 'enabled', released=stop)

        async def refresh():
            play_button.visible = self.runtime.automator.is_stopped
            pause_button.visible = self.runtime.automator.is_running
            resume_button.visible = self.runtime.automator.is_paused
            play_button.view.disable = default_automation is None or not self.runtime.automator.enabled
            stop_button.view.disable = self.runtime.automator.is_stopped
            if app_conntrols is not None:
                before = ' '.join([str(b) for b in app_conntrols.buttons.values()])
                app_conntrols.buttons['play'].visible = play_button.visible
                app_conntrols.buttons['pause'].visible = pause_button.visible
                app_conntrols.buttons['resume'].visible = resume_button.visible
                app_conntrols.buttons['play'].state = 'disabled' if play_button.view.disable else 'enabled'
                app_conntrols.buttons['stop'].state = 'disabled' if stop_button.view.disable else 'enabled'
                if ' '.join([str(b) for b in app_conntrols.buttons.values()]) != before:
                    await app_conntrols.sync()
        self.ui.timer(update_interval, refresh)
