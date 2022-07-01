from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Callable

from rosys.event import Event
from rosys.hardware.communication import Communication
from rosys.task_logger import create_task


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
    APP_CONNECTED = Event('called after app connected via bluetooth; use this to refresh infos or similar')

    def __init__(self, communication: Communication) -> None:
        self.communication = communication
        self.main_buttons: dict[str, AppButton] = {}
        self.extra_buttons: dict[str, AppButton] = {}

    async def set_info(self, msg: str) -> None:
        '''replace constantly shown info text on mobile device'''
        await self.communication.send(f'bluetooth.send("PUT /info {msg}")')

    async def notify(self, msg: str) -> None:
        '''show notification as Snackbar message on mobile device'''
        await self.communication.send(f'bluetooth.send("POST /notification {msg}")')

    def parse(self, line: str) -> None:
        if line.startswith('"'):
            line = line[1:-1]
        if line.startswith('app: '):
            line = line[5:]
            if line == 'connected':
                create_task(self.sync(), name='sync app')
                self.APP_CONNECTED.emit()
            elif line.startswith('PUT /button/') and '/action' in line:
                # line: "PUT /button/main/my_button/action pressed"
                _, path, action = line.split(' ')
                _, group, name, _ = path.split('/')
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
                    await self.communication.send(cmd)
        await run('main', self.main_buttons)
        await run('extra', self.extra_buttons)

    async def _invoke(self, callback: Callable):
        if inspect.iscoroutinefunction(callback):
            create_task(callback(), name='app_controls._invoke')
        else:
            callback()
