from __future__ import annotations

import inspect
from collections.abc import Callable
from dataclasses import dataclass

from .. import rosys
from ..event import Event
from ..hardware import RobotBrain
from .automator import Automator


@dataclass
class AppButton:
    icon: str
    state: str = 'enabled'
    visible: bool = True
    pressed: Callable[[], None] = lambda: None
    released: Callable[[], None] = lambda: None
    canceled: Callable[[], None] = lambda: None

    def get_properties(self) -> list[str]:
        return [
            f'/icon {self.icon}',
            f'/state {self.state}',
            f'/visible {"true" if self.visible else "false"}',
        ]

    def __str__(self) -> str:
        return f'AppButton({self.icon},{self.state},{self.visible})'


class AppControls:
    """The AppControls module enables the connection with a mobile-app-based user interface.

    It uses a given RobotBrain object to communicate with [Lizard](https://lizard.dev/) running on a microcontroller
    and in turn being connected to a mobile app via Bluetooth Low Energy.
    It displays buttons to control a given automator.
    """

    def __init__(self, robot_brain: RobotBrain, automator: Automator) -> None:
        self.APP_CONNECTED = Event[[]]()
        """an app connected via bluetooth (used to refresh information or similar)"""

        self.robot_brain = robot_brain
        self.automator = automator

        self.main_buttons: dict[str, AppButton] = {
            'play': AppButton('play_arrow', released=automator.start),
            'pause': AppButton('pause', released=lambda: automator.pause(because='pause button was pressed')),
            'resume': AppButton('play_arrow', released=automator.resume),
            'stop': AppButton('stop', released=lambda: automator.stop(because='stop button was pressed')),
        }
        self.extra_buttons: dict[str, AppButton] = {}

        rosys.on_startup(self.sync)
        rosys.on_shutdown(self.clear)
        rosys.on_repeat(self.refresh, 0.1)
        rosys.NEW_NOTIFICATION.register(self.notify)
        robot_brain.LINE_RECEIVED.register(self.parse)

    async def refresh(self) -> None:
        if not self.main_buttons:  # happens on shutdown
            return
        before = ' '.join(str(b) for b in self.main_buttons.values())
        self.main_buttons['play'].visible = self.automator.is_stopped
        self.main_buttons['pause'].visible = self.automator.is_running
        self.main_buttons['resume'].visible = self.automator.is_paused
        self.main_buttons['play'].state = 'disabled' if self.automator.default_automation is None or not self.automator.enabled else 'enabled'
        self.main_buttons['stop'].state = 'disabled' if self.automator.is_stopped else 'enabled'
        after = ' '.join(str(b) for b in self.main_buttons.values())
        if after != before:
            await self.sync()

    async def set_info(self, msg: str) -> None:
        """replace constantly shown info text on mobile device"""
        await self.robot_brain.send(f'bluetooth.send("PUT /info {msg}")')

    async def notify(self, msg: str) -> None:
        """show notification as Snackbar message on mobile device"""
        await self.robot_brain.send(f'bluetooth.send("POST /notification {msg}")')

    def parse(self, line: str) -> None:
        if line.startswith('"'):
            line = line[1:-1]
        if line.startswith('app: '):
            line = line[5:]
            if line == 'connected':
                rosys.background_tasks.create(self.sync(), name='sync app')
                self.APP_CONNECTED.emit()
            elif line.startswith('PUT /button/') and '/action' in line:
                # line: "PUT /button/main/my_button/action pressed"
                _, path, action = line.split(' ')
                _, group, name, _ = path[1:].split('/')
                buttons = self.main_buttons if group == 'main' else self.extra_buttons
                if action == 'pressed':
                    self._invoke(buttons[name].pressed)
                if action == 'released':
                    self._invoke(buttons[name].released)
                if action == 'canceled':
                    self._invoke(buttons[name].canceled)

    async def sync(self):
        await self.send('PUT', lambda b: b.get_properties())

    async def clear(self):
        await self.send('DELETE')
        self.main_buttons.clear()

    async def send(self, method: str, get_properties: Callable[[AppButton], list[str]] = lambda _: ['']):
        async def run(group: str, buttons: dict[str, AppButton]) -> None:
            for name, button in buttons.items():
                for prop in get_properties(button):
                    cmd = f'bluetooth.send("{method} /button/{group}/{name}{prop}")'
                    await self.robot_brain.send(cmd)
        await run('main', self.main_buttons)
        await run('extra', self.extra_buttons)

    def _invoke(self, callback: Callable):
        if inspect.iscoroutinefunction(callback):
            rosys.background_tasks.create(callback(), name='app_controls._invoke')
        else:
            callback()
