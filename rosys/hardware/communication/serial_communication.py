import os
from typing import Optional

import rosys
import serial
from nicegui import ui
from nicegui.events import ValueChangeEventArguments

from .communication import Communication


class SerialCommunication(Communication):
    baudrate: int = 115200
    log_io: bool = False

    def __init__(self) -> None:
        super().__init__()
        self.device_path = self.get_device_path()
        if self.device_path is None:
            raise Exception('No serial port found')
        self.log.debug(f'connecting serial on {self.device_path} with baudrate {self.baudrate}')
        self.serial = serial.Serial(self.device_path, self.baudrate)
        self.buffer = ''

    @staticmethod
    def is_possible() -> bool:
        return SerialCommunication.get_device_path() is not None

    @staticmethod
    def get_device_path() -> Optional[str]:
        device_paths = [
            '/dev/ttyUSB0',
            '/dev/ttyTHS1',
            '/dev/tty.SLAB_USBtoUART',
            '/dev/esp',
        ]
        for device_path in device_paths:
            if os.path.exists(device_path) and os.stat(device_path).st_gid > 0:
                return device_path

    def connect(self) -> None:
        if not self.serial.isOpen():
            self.serial.open()
            self.log.debug(f'reconnected serial on {self.device_path}')

    def disconnect(self) -> None:
        if self.serial.isOpen():
            self.serial.close()
            self.log.debug(f'disconnected serial on {self.device_path}')

    async def read(self) -> Optional[str]:
        if not self.serial.isOpen():
            return
        s = self.serial.read_all()
        s = s.decode()
        self.buffer += s
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\r\n', 1)
            if self.log_io:
                self.log.debug(f'read: {line}')
            return line

    async def send(self, line: str) -> None:
        if not self.serial.isOpen():
            return
        self.serial.write(f'{line}\n'.encode())
        if self.log_io:
            self.log.debug(f'send: {line}')

    def debug_ui(self) -> None:
        super().debug_ui()

        async def submit_input() -> None:
            await self.send(input.value)
            input.value = ''

        def toggle(e: ValueChangeEventArguments) -> None:
            if e.value:
                self.connect()
                rosys.notify('connected to Lizard')
            else:
                self.disconnect()
                rosys.notify('disconnected from Lizard')

        ui.switch('Serial Communication', value=self.serial.isOpen(), on_change=toggle)
        ui.switch('Serial Logging').bind_value(self, 'log_io')
        input = ui.input(on_change=submit_input)
