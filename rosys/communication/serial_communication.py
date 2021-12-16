from typing import Optional
import asyncio
import os
import serial
from .checksum import augment, check
from .communication import Communication


class SerialCommunication(Communication):
    device_path: str = '/dev/esp'
    baudrate: int = 115200

    def __init__(self):
        self.serial = serial.Serial(self.device_path, self.baudrate)
        self.buffer = ''

    @classmethod
    def is_possible(cls) -> bool:
        return os.path.exists(cls.device_path) and os.stat(cls.device_path).st_gid > 0

    def connect(self):
        if not self.serial.isOpen():
            self.serial.open()

    def disconnect(self):
        if self.serial.isOpen():
            self.serial.close()

    async def read(self) -> Optional[str]:
        if not self.serial.isOpen():
            return
        self.buffer += self.serial.read_all().decode()
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\r\n', 1)
            return check(line)

    async def send_async(self, line: str):
        self.serial.write(f'{augment(line)}\n'.encode())
        await asyncio.sleep(0)
