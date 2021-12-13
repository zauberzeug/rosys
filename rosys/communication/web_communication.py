from typing import Optional
import socketio
import socket
from collections import deque
from .communication import Communication


class WebCommunication(Communication):
    '''Remote connection to the Robot Brains Esp.

    This makes it possible to keep developing on your fast computer
    while communicating with the hardware components connected to a physical Robot Brain.
    '''

    ip: str = '192.168.43.1'
    port: int = 8081
    host: str = f'ws://{ip}:{port}'

    def __init__(self):
        self.buffer: deque[str] = deque()
        self.sio = socketio.AsyncClient()

        @self.sio.event
        def connect():
            assert self.sio.transport() == 'websocket'

        @self.sio.event
        def read(msg):
            self.buffer.append(msg)

    @classmethod
    def is_possible(cls) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((cls.ip, cls.port))
            s.shutdown(2)
        except Exception as e:
            print(40, e)
            return False

        try:
            sio = socketio.Client(reconnection=False)

            @sio.event
            def connect():
                sio.disconnect()

            sio.connect(cls.host)
        except Exception as e:
            print(51, e)
            return False

        return True

    async def read(self) -> Optional[str]:
        if not self.sio.connected:
            await self.sio.connect(self.host)
        if self.buffer:
            return self.buffer.pop()

    async def send_async(self, line: str):
        await self.sio.emit('write', line)

    async def tear_down(self):
        await self.sio.disconnect()
