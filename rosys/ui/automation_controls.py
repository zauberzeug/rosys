import inspect
from typing import Awaitable, Callable, Optional, Union

from nicegui.ui import Ui
from rosys.hardware import AppButton, AppControls

from .. import Runtime
from .settings import Settings as settings


class AutomationControls:
    ui: Ui
    runtime: Runtime

    def __init__(self, *,
                 default_automation: Optional[Awaitable] = None,
                 can_start: Optional[Callable[[], Union[bool, Awaitable[bool]]]] = None,
                 app_controls: AppControls = None,
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

        play_button = self.ui.button(on_click=start).props('icon=play_arrow unelevated').tooltip('start automation')
        pause_button = self.ui.button(on_click=pause).props('icon=pause outline').tooltip('stop automation')
        resume_button = self.ui.button(on_click=resume).props('icon=play_arrow outline').tooltip('resume automation')
        stop_button = self.ui.button(on_click=stop).props('icon=stop outline').tooltip('stop automation')

        if app_controls is not None:
            app_controls.main_buttons['play'] = AppButton('play_arrow', released=start)
            app_controls.main_buttons['pause'] = AppButton('pause', released=pause)
            app_controls.main_buttons['resume'] = AppButton('play_arrow', released=resume)
            app_controls.main_buttons['stop'] = AppButton('stop', released=stop)

        async def refresh():
            play_button.visible = self.runtime.automator.is_stopped
            pause_button.visible = self.runtime.automator.is_running
            resume_button.visible = self.runtime.automator.is_paused
            play_button.view.disable = default_automation is None or not self.runtime.automator.enabled
            stop_button.view.disable = self.runtime.automator.is_stopped
            if app_controls is not None:
                before = ' '.join(str(b) for b in app_controls.main_buttons.values())
                app_controls.main_buttons['play'].visible = play_button.visible
                app_controls.main_buttons['pause'].visible = pause_button.visible
                app_controls.main_buttons['resume'].visible = resume_button.visible
                app_controls.main_buttons['play'].state = 'disabled' if play_button.view.disable else 'enabled'
                app_controls.main_buttons['stop'].state = 'disabled' if stop_button.view.disable else 'enabled'
                after = ' '.join(str(b) for b in app_controls.main_buttons.values())
                if after != before:
                    await app_controls.sync()
        self.ui.timer(settings.update_interval, refresh)
