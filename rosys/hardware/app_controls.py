from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Callable

import rosys


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


class AppControls():

    def __init__(self, robot_brain: rosys.hardware.RobotBrain) -> None:
        self.robot_brain = robot_brain
        self.main_buttons: dict[str, AppButton] = {}
        self.extra_buttons: dict[str, AppButton] = {}

    async def set_info(self, msg: str) -> None:
        '''replace constantly shown info text on mobile defice'''
        await self.robot_brain.send(f'bluetooth.send("PUT /info {msg}")')

    async def notify(self, msg: str) -> None:
        '''show notification as Snackbar message on mobile device'''
        await self.robot_brain.send(f'bluetooth.send("POST /notification {msg}")')

    async def parse(self, line: str) -> None:
        if line.startswith('"'):
            line = line[1:-1]
        if line.startswith('app: '):
            line = line[5:]
            if line == 'connected':
                await self.sync()
                rosys.event.emit(rosys.event.Id.APP_CONNECTED)
            elif line.startswith('PUT /button/') and '/action' in line:
                cmd = line.split(' ')
                path = cmd[1].split('/')
                name = path[-2]
                group = path[-3]
                action = cmd[2]
                buttons = self.main_buttons if group == 'main' else self.extra_buttons
                if action == 'pressed':
                    await self._invoke(buttons[name].pressed)
                if action == 'released':
                    await self._invoke(buttons[name].released)
                if action == 'canceled':
                    await self._invoke(buttons[name].canceled)

    async def sync(self):
        await self.send('PUT', lambda b: b.get_properties())

    async def clear(self):
        await self.send('DELETE')
        self.main_buttons.clear()

    async def send(self, method: str, get_properties: Callable[[AppButton], list[str]] = lambda b: ['']):
        async def run(group: str, buttons: list[AppButton]) -> None:
            for name, button in buttons.items():
                for prop in get_properties(button):
                    cmd = f'bluetooth.send("{method} /button/{group}/{name}{prop}")'
                    await self.robot_brain.send(cmd)
        await run('main', self.main_buttons)
        await run('exra', self.extra_buttons)

    async def _invoke(self, callback: Callable):
        if inspect.iscoroutinefunction(callback):
            await callback()
        else:
            callback()
