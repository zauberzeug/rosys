from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Callable

import rosys


@dataclass
class AppButton:
    icon: str
    state: str
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
        self.buttons: dict[str, AppButton] = {}

    async def parse(self, line: str):
        if line.startswith('"'):
            line = line[1:-1]
        if line.startswith('app: '):
            line = line[5:]
            if line == 'connected':
                await self.sync()
            elif line.startswith('PUT /control/button/') and '/action' in line:
                name = line[20:line.index('/action')]
                action = line.split(' ')[-1]
                if action == 'pressed':
                    await self._invoke(self.buttons[name].pressed)
                if action == 'released':
                    await self._invoke(self.buttons[name].released)
                if action == 'canceled':
                    await self._invoke(self.buttons[name].canceled)

    async def sync(self):
        await self.send('PUT', lambda b: b.get_properties())

    async def clear(self):
        await self.send('DELETE')
        self.buttons.clear()

    async def send(self, method: str, get_properties: Callable[[AppButton], list[str]] = lambda b: ['']):
        for name, button in self.buttons.items():
            for prop in get_properties(button):
                cmd = f'bluetooth.send("{method} /control/button/{name}{prop}")'
                await self.robot_brain.send(cmd)

    async def _invoke(self, callback: Callable):
        if inspect.iscoroutinefunction(callback):
            await callback()
        else:
            callback()
