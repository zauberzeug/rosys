import logging
import os
from collections import deque
from typing import ClassVar

import serial
from nicegui import ui
from nicegui.events import ValueChangeEventArguments
from serial.serialutil import SerialException

from ... import rosys
from .communication import Communication


class SerialCommunication(Communication):
    """This module implements a communication via a serial device with a given baud rate.

    It contains a list of search paths for finding the serial device.
    """

    search_paths: ClassVar[list[str]] = [
        '/dev/tty.SLAB_USBtoUART',
        '/dev/ttyTHS1',
        '/dev/ttyUSB0',
    ]

    def __init__(self, *, device_path: str | None = None, baud_rate: int = 115200) -> None:
        super().__init__()
        self.device_path = self.get_device_path() if device_path is None else device_path
        if self.device_path is None:
            raise FileNotFoundError('No serial port found')
        self.log.debug('connecting serial on %s with baud rate %s', self.device_path, baud_rate)
        self.serial = serial.Serial(self.device_path, baud_rate)
        self.buffer = ''
        self.undo_queue: deque[str] = deque(maxlen=100)
        self.redo_queue: deque[str] = deque(maxlen=100)

    @staticmethod
    def is_possible() -> bool:
        device_path = SerialCommunication.get_device_path()
        if device_path is None:
            return False
        try:
            with serial.Serial(device_path):
                return True  # Successfully opened the port
        except SerialException:
            return False

    @staticmethod
    def get_device_path() -> str | None:
        for device_path in SerialCommunication.search_paths:
            if os.path.exists(device_path) and os.stat(device_path).st_gid > 0:
                return device_path
        return None

    def connect(self) -> None:
        if not self.serial.isOpen():
            self.serial.open()
            self.log.debug('reconnected serial on %s', self.device_path)

    def disconnect(self) -> None:
        if self.serial.isOpen():
            self.serial.close()
            self.log.debug('disconnected serial on %s', self.device_path)

    async def read(self) -> str | None:
        if not self.serial.isOpen():
            return None
        self.buffer += self.serial.read_all().decode(errors='replace')
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\r\n', 1)
            self.log.debug('read: %s', line)
            return line
        return None

    async def send(self, msg: str) -> None:
        if not self.serial.isOpen():
            return
        self.serial.write(f'{msg}\n'.encode())
        self.log.debug('send: %s', msg)

    def debug_ui(self) -> None:
        super().debug_ui()

        async def input_enter() -> None:
            await self.send(command.value)
            while self.redo_queue:
                self.undo_queue.append(self.redo_queue.pop())
            self.undo_queue.append(command.value)
            command.value = ''

        def input_up() -> None:
            if self.undo_queue:
                if command.value:
                    self.redo_queue.append(command.value)
                command.value = self.undo_queue.pop()

        def input_down() -> None:
            if command.value:
                self.undo_queue.append(command.value)
            command.value = self.redo_queue.pop() if self.redo_queue else ''

        def toggle(e: ValueChangeEventArguments) -> None:
            if e.value:
                self.connect()
                rosys.notify('connected to Lizard')
            else:
                self.disconnect()
                rosys.notify('disconnected from Lizard')

        ui.switch('Serial Communication', value=self.serial.isOpen(), on_change=toggle)
        ui.switch('Serial Logging', value=self.log.getEffectiveLevel() <= logging.DEBUG,
                  on_change=lambda e: self.log.setLevel(logging.DEBUG if e.value else logging.INFO))
        command = ui.input('Serial Command')
        command.on('keydown.enter', input_enter)
        command.on('keydown.up', input_up)
        command.on('keydown.down', input_down)
