import socketio
from datetime import datetime, timedelta
from ..world import BoxDetection, PointDetection
from .actor import Actor
from .. import event
from ..world import Frame


class Detector(Actor):
    interval: float = 0.02

    def __init__(self):
        super().__init__()
        self.sio = socketio.AsyncClient()
        self.is_connected = None

        @self.sio.event
        def connect():
            self.is_connected = True
            assert self.sio.transport() == 'websocket'

        @self.sio.event
        def connect_error(data):
            self.is_connected = False

        @self.sio.event
        def disconnect():
            self.is_connected = False

    async def connect(self, reconnect_delay: float = 3):
        try:
            self.log.info('connecting to detector')
            await self.sio.connect('ws://localhost:8004', socketio_path='/ws/socket.io')
            self.log.info('connected successfully')
            self.is_connected = True
        except:
            self.log.exception('connection failed; trying again')
            self.is_connected = None
            await self.sleep(reconnect_delay)

    async def step(self):
        if self.is_connected is None:
            await self.connect()

    async def detect(self, frame: Frame, group: str = 'default_group') -> Frame:
        if not self.is_connected:
            return
        try:
            result = await self.sio.call('detect', {'image': frame.data, 'mac': group})
            box_detections = [BoxDetection.parse_obj(d) for d in result.get('box_detections', [])]
            point_detections = [PointDetection.parse_obj(d) for d in result.get('point_detections', [])]
            frame.detections = box_detections + point_detections
        except:
            self.log.exception(f'could not detect {frame} in {group}')
        else:
            event.emit(event.Id.NEW_DETECTIONS, frame)
            return frame

    async def upload(self, frame: Frame, group: str = 'default_group'):
        try:
            await self.sio.emit('upload', {'image': frame.data, 'mac': group})
        except:
            self.log.exception(f'could not upload  {frame} in {group}')

    def __str__(self) -> str:
        state = {
            None: 'starting',
            True: 'connected',
            False: 'reconnecting',
        }[self.is_connected]
        return f'{type(self).__name__} ({state})'
