import socketio
from .esp import Esp
from ..world.world import World


class WebEsp(Esp):
    interval: float = 0.01

    def __init__(self):
        super().__init__()
        self.host = 'ws://localhost:80'
        # let's try to establish a connection to the esp proxy synchronously...
        socketio.Client(reconnection=False).connect(self.host)

        # ... ok, we passed without exception -> creating async client
        self.sio = socketio.AsyncClient()

        @self.sio.event
        def connect():
            assert self.sio.transport() == 'websocket'
            self.log.info('connected to esp proxy')

        @self.sio.event
        def read(data):
            self.log.info(f'received "{data}"')

    async def step(self, world: World):
        if not self.sio.connected:
            await self.sio.connect(self.host)

    async def send_async(self, line):
        await self.sio.emit('write', line)

    async def tear_down(self):
        await self.sio.disconnect()
