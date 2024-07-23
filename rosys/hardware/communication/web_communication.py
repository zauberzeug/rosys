from collections import deque

import socketio

from .communication import Communication


class WebCommunication(Communication):
    """Remote connection to the Robot Brain's ESP.

    This makes it possible to keep developing on your fast computer
    while communicating with the hardware components connected to a physical Robot Brain.
    """
    ip: str = '192.168.43.1'
    port: int = 8081
    host: str = f'ws://{ip}:{port}'

    def __init__(self) -> None:
        super().__init__()
        self.buffer: deque[str] = deque()
        self.sio = socketio.AsyncClient()

        @self.sio.event
        def connect() -> None:
            assert self.sio.transport() == 'websocket'

        @self.sio.event
        def read(msg) -> None:
            self.buffer.append(msg)

    @classmethod
    def is_possible(cls) -> bool:
        try:
            sio = socketio.Client(reconnection=False)
            sio.on('connect', sio.disconnect)
            sio.connect(cls.host)
            return True
        except Exception:
            return False

    async def read(self) -> str | None:
        if not self.sio.connected:
            await self.sio.connect(self.host)
        if self.buffer:
            return self.buffer.pop()
        return None

    async def send(self, msg: str) -> None:
        await self.sio.emit('write', msg)

    async def tear_down(self) -> None:
        await self.sio.disconnect()
