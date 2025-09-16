from nicegui import ui

from ..rosys import config
from .automator import Automator


class AutomationControls:
    """This UI element contains start/stop/pause/resume buttons for controlling a given automator.

    See [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) for a simple example of the automation controls.
    """

    def __init__(self, automator: Automator) -> None:
        play_button = ui.button(on_click=automator.start) \
            .props('icon=play_arrow unelevated').tooltip('start automation')
        pause_button = ui.button(on_click=lambda: automator.pause(because='pause button was pressed')) \
            .props('icon=pause outline').tooltip('pause automation')
        resume_button = ui.button(on_click=automator.resume) \
            .props('icon=play_arrow outline').tooltip('resume automation')
        stop_button = ui.button(on_click=lambda: automator.stop(because='stop button was pressed')) \
            .props('icon=stop outline').tooltip('stop automation')

        def refresh() -> None:
            uninterruptible_is_running = automator.is_pausing or automator.is_stopping
            play_button.visible = automator.is_stopped
            play_button.enabled = play_button.visible and automator.default_automation is not None and automator.enabled
            pause_button.visible = automator.is_running
            pause_button.enabled = pause_button.visible and not uninterruptible_is_running
            resume_button.visible = automator.is_paused
            resume_button.enabled = resume_button.visible and automator.enabled and not uninterruptible_is_running
            stop_button.enabled = not automator.is_stopped and not uninterruptible_is_running

        ui.timer(config.ui_update_interval, refresh)
