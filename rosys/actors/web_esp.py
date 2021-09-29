import socketio
from .esp import Esp
from ..world.world import World


class WebEsp(Esp):
    '''Remote connection to the Robot Brains Esp.

    This makes it possible to keep developing on your fast computer 
    while communicating with the hardware componens connected to a physical Robot Brain.
    '''
    interval: float = 0.01

    def __init__(self, world: World):
        super().__init__()
        self.host = 'ws://192.168.43.1:80'
        # let's try to establish a connection to the esp proxy synchronously...
        self._try_socketio()
        # ... ok, we passed without exception -> creating async client
        self.sio = socketio.AsyncClient()

        @self.sio.event
        def connect():
            assert self.sio.transport() == 'websocket'
            self.log.info('connected to esp proxy')

        @self.sio.event
        def read(data):
            self.parse(data + '\n', world)

    async def step(self, world: World):
        if not self.sio.connected:
            await self.sio.connect(self.host)

    async def send_async(self, line):
        await self.sio.emit('write', line)

    async def tear_down(self):
        await self.sio.disconnect()

    def _try_socketio(self):
        sio = socketio.Client(reconnection=False)

        @sio.event
        def connect():
            sio.disconnect()

        sio.connect(self.host)
