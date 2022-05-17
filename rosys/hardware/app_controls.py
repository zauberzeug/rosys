from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import rosys


@dataclass
class Button:
    icon: str
    state: str

    def get_properties(self) -> list[str]:
        return [
            f'/icon {self.icon}',
            f'/state {self.state}'
        ]


class AppControls():

    def __init__(self, automator: rosys.actors.Automator, robot_brain: rosys.hardware.RobotBrain) -> None:
        self.robot_brain = robot_brain
        self.automator = automator
        self.buttons: dict[str, Button] = {}
        if self.automator is not None:
            self.buttons['playpause'] = Button('play_arrow', 'enabled')
            self.buttons['stop'] = Button('stop', 'enabled')

    async def parse(self, line: str):
        if line.startswith('"'):
            line = line[1:-1]
        if line.startswith('app: '):
            line = line[5:]
            print(line, flush=True)
            if line == 'connected':
                await self.sync()

    async def sync(self):
        if self.automator is not None:
            if not self.automator.enabled:
                self.buttons['playpause'].state = self.buttons['stop'].state = 'disabled'
            elif self.automator.is_stopped:
                self.buttons['stop'].state = 'disabled'
            elif self.automator.is_running:
                self.buttons['playpause'].icon = 'pause'
        await self.send('PUT', lambda b: b.get_properties())

    async def clear(self):
        await self.send('DELETE')
        self.buttons.clear()

    async def send(self, method: str, get_properties: Callable[[Button], list[str]] = lambda b: ['']):
        for name, button in self.buttons.items():
            for prop in get_properties(button):
                cmd = f'bluetooth.send("{method} /control/button/{name}{prop}")'
                print(f'sending {cmd}')
                await self.robot_brain.send(cmd)
