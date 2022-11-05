import logging
import os
import sys
from typing import Optional

import rosys
from nicegui import ui
from rosys import task_logger

from ..event import Event
from .communication import Communication, SerialCommunication


class RobotBrain:
    '''This module manages the communication with a [Zauberzeug Robot Brain](https://zauberzeug.com/robot-brain.html).

    It expects a communication object, which is used for the actual read and write operations.
    Besides providing some basic methods like configuring or restarting the microcontroller, it augments and verifies checksums for each message.
    '''

    def __init__(self, communication: Communication, lizard_startup: str = 'lizard.txt') -> None:
        self.LINE_RECEIVED = Event()
        '''a line has been received from the microcontroller (argument: line as string)'''

        self.communication = communication
        self.lizard_startup = lizard_startup
        self.waiting_list: dict[str, Optional[str]] = {}
        self.lizard_version: Optional[str] = None
        self.clock_offset: Optional[float] = None
        self.hardware_time: Optional[float] = None
        self.flash_params: list[str] = []
        self.log = logging.getLogger('rosys.robot_rain')

    async def configure(self) -> None:
        self.log.info('Configuring...')
        await self.send(f'!-')
        with open(self.lizard_startup) as f:
            for line in f.read().splitlines():
                await self.send(f'!+{line}')
        await self.send(f'!.')
        await self.restart()

    async def restart(self) -> None:
        await self.send(f'core.restart()')

    async def read_lines(self) -> list[tuple[float, str]]:
        lines: list[tuple[float, str]] = []
        millis = None
        while True:
            unchecked = await self.communication.read()
            line = check(unchecked)
            if not line:
                break
            words = line.split()
            if not words:
                continue
            first = words.pop(0)
            if first in self.waiting_list:
                self.waiting_list[first] = line
            if first == 'core':
                millis = float(words.pop(0))
                if self.clock_offset is None:
                    continue
                self.hardware_time = millis / 1000 + self.clock_offset
            self.LINE_RECEIVED.emit(line)
            lines.append((self.hardware_time, line))
        if millis is not None:
            self.clock_offset = rosys.time() - millis / 1000
        return lines

    async def send(self, msg: str) -> None:
        await self.communication.send(augment(msg))

    async def send_and_await(self, msg: str, ack: str, *, timeout: float = float('inf')) -> Optional[str]:
        self.waiting_list[ack] = None
        await self.send(msg)
        t0 = rosys.time()
        while self.waiting_list.get(ack) is None and rosys.time() < t0 + timeout:
            await rosys.sleep(0.1)
        return self.waiting_list.pop(ack) if ack in self.waiting_list else None

    async def flash(self) -> None:
        assert isinstance(self.communication, SerialCommunication)
        rosys.notify(f'Installing Lizard firmware {self.available_lizard_version}')
        self.communication.disconnect()
        output = await rosys.run.sh(['./flash.py'] + self.flash_params, timeout=None, working_dir=os.path.expanduser('~/.lizard'))
        self.log.info(f'flashed Lizard:\n {output}')
        self.communication.connect()
        await self.configure()
        rosys.notify(f'Installed Lizard firmware {self.lizard_version}')

    def developer_ui(self) -> None:
        ui.label(f'Lizard').bind_text_from(self, 'lizard_version', backward=lambda x: f'Lizard ({x or "?"})')
        ui.button('Configure', on_click=self.configure).props('outline')
        ui.button('Restart', on_click=self.restart).props('outline')
        ui.button(f'Flash ({self.available_lizard_version})', on_click=self.flash).props('outline')
        ui.button('Enable', on_click=self.enable_esp).props('outline')
        ui.label().bind_text_from(self, 'clock_offset', lambda offset: f'Clock offset: {offset or 0:.3f} s')

    async def ensure_lizard_version(self) -> None:
        await self.determine_lizard_version()
        if self.lizard_version != self.available_lizard_version:
            task_logger.create_task(self.flash)
        await self.determine_lizard_version()

    async def determine_lizard_version(self) -> str:
        while (response := await self.send_and_await('core.info()', 'lizard', timeout=1)) is None:
            self.log.warning('Could not get Lizard version')
            await rosys.sleep(0.1)
            continue
        self.lizard_version = response.split()[-1].split('-')[0][1:]

    def enable_esp(self) -> None:
        try:
            sys.path.insert(1, os.path.expanduser('~/.lizard'))
            from esp import Esp
            esp = Esp()
            with esp.pin_config():
                esp.activate()
        except:
            self.log.exception('Could not enable ESP')

    def __del__(self) -> None:
        self.communication.disconnect()

    def __repr__(self) -> str:
        return f'<RobotBrain {self.communication}>'

    @property
    def available_lizard_version(self) -> str:
        with open(os.path.expanduser('~/.lizard/build/lizard.bin'), 'rb') as f:
            head = f.read(150).decode('utf-8', 'backslashreplace')
            version = head.split(' ')[3]
            return version[1:version.find('lizard')].strip()


def augment(line: str) -> str:
    checksum = 0
    for c in line:
        checksum ^= ord(c)
    return f'{line}@{checksum:02x}'


def check(line: Optional[str]) -> str:
    if line is None:
        return ""
    if line[-3:-2] == '@':
        check = int(line[-2:], 16)
        line = line[:-3]
        checksum = 0
        for c in line:
            checksum ^= ord(c)
        if checksum != check:
            return None
    return line
