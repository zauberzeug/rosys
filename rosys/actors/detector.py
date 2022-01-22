import socketio
from ..world import BoxDetection, PointDetection
from .actor import Actor
from .. import event, task_logger
from ..world import Frame


class Detector(Actor):
    interval: float = 0

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

        if self.world.upload_queue:
            task_logger.create_task(self.upload(self.world.upload_queue.pop(0)), name='upload_frame')

        detecting_cameras = [c for c in self.world.cameras.values() if c.detect]
        if not detecting_cameras:
            await self.sleep(0.02)
            return

        for camera in detecting_cameras:
            frame = camera.frames[-1]
            await self.detect(frame)

    async def detect(self, frame: Frame) -> Frame:
        if not self.is_connected:
            return
        try:
            result = await self.sio.call('detect', {'image': frame.data, 'mac': frame.camera_id})
            box_detections = [BoxDetection.parse_obj(d) for d in result.get('box_detections', [])]
            point_detections = [PointDetection.parse_obj(d) for d in result.get('point_detections', [])]
            frame.detections = box_detections + point_detections
        except:
            self.log.exception(f'could not detect {frame}')
        else:
            event.emit(event.Id.NEW_DETECTIONS, frame)
            return frame

    async def upload(self, frame: Frame):
        try:
            await self.sio.emit('upload', {'image': frame.data, 'mac': frame.camera_id})
        except:
            self.log.exception(f'could not upload  {frame}')

    def __str__(self) -> str:
        state = {
            None: 'starting',
            True: 'connected',
            False: 'reconnecting',
        }[self.is_connected]
        return f'{type(self).__name__} ({state})'
