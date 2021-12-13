from typing import Optional
import os
import aioserial
from .communication import Communication
from .checksum import augment, check


class SerialCommunication(Communication):
    device_path: str = '/dev/esp'
    baudrate: int = 115200

    def __init__(self):
        self.aioserial = aioserial.AioSerial(self.device_path, baudrate=self.baudrate)
        self.buffer = ''

    @classmethod
    def is_possible(cls) -> bool:
        return os.path.exists(cls.device_path) and os.stat(cls.device_path).st_gid > 0

    def connect(self):
        if not self.aioserial.isOpen():
            self.aioserial.open()

    def disconnect(self):
        if self.aioserial.isOpen():
            self.aioserial.close()

    async def read(self) -> Optional[str]:
        self.buffer += self.aioserial.read_all().decode()
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\r\n', 1)
            return check(line)

    async def send_async(self, line: str):
        await self.aioserial.write_async(f'{augment(line)}\n'.encode())
