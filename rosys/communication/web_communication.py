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

    host: str = 'ws://192.168.43.1:80'

    def __init__(self):
        self.sio = socketio.AsyncClient()
        self.buffer: deque[str] = deque()

        @self.sio.event
        def connect():
            assert self.sio.transport() == 'websocket'
            self.log.info('connected to esp proxy')

        @self.sio.event
        def read(msg):
            self.buffer.append(msg)

    @classmethod
    def is_possible(cls) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(cls.host.replace('ws://', '',).split(':'))
            s.shutdown(2)

            sio = socketio.Client(reconnection=False,)

            @sio.event
            def connect():
                sio.disconnect()

            sio.connect(cls.host)
            return True
        except:
            return False

    def read(self) -> Optional[str]:
        if self.buffer:
            return self.buffer.pop()

    async def send_async(self, line: str):
        await self.sio.emit('write', line)
