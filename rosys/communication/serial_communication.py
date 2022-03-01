from typing import Optional
import os
import serial
import logging
from .communication import Communication

log = logging.getLogger('rosys.hardware.serial_communication')


class SerialCommunication(Communication):
    baudrate: int = 115200

    def __init__(self):
        super().__init__()
        self.device_path = SerialCommunication.get_device_path()
        if self.device_path is None:
            raise Exception('No serial port found')
        self.log.debug(f'connecting serial on {self.device_path} with baudrate {self.baudrate}')
        self.serial = serial.Serial(self.device_path, self.baudrate)
        self.buffer = ''

    @classmethod
    def is_possible(cls) -> bool:
        return cls.get_device_path() is not None

    @classmethod
    def get_device_path(cls) -> Optional[str]:
        device_paths = [
            '/dev/ttyTHS1',
            '/dev/ttyUSB0',
            '/dev/tty.SLAB_USBtoUART',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            '/dev/esp',
        ]
        for device_path in device_paths:
            if os.path.exists(device_path) and os.stat(device_path).st_gid > 0:
                return device_path

    def connect(self):
        if not self.serial.isOpen():
            self.serial.open()
            self.log.debug(f'reconnected serial on {self.device_path}')

    def disconnect(self):
        if self.serial.isOpen():
            self.serial.close()
            self.log.debug(f'disconnected serial on {self.device_path}')

    async def read(self) -> Optional[str]:
        if not self.serial.isOpen():
            return
        self.buffer += self.serial.read_all().decode()
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\r\n', 1)
            self.log.debug(f'read: {line}')
            return line

    async def send_async(self, line: str):
        self.serial.write(f'{line}\n'.encode())
        self.log.debug(f'send: {line}')
