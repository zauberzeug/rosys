import socketio
from datetime import datetime, timedelta
from .. import event, sleep, task_logger
from ..world import BoxDetection, Image, PointDetection
from .actor import Actor


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
            await sleep(reconnect_delay)

    async def step(self):
        if self.is_connected is None:
            await self.connect()

        if not self.is_connected:
            return

        for image in self.world.upload.queue:
            if datetime.now() < self.world.upload.last_upload + timedelta(minutes=self.world.upload.minimal_minutes_between_uploads):
                break
            if image.data:
                self.world.upload.last_upload = datetime.now()
                self.world.upload.queue.clear()  # because old images should not be uploaded later when the robot is inactive
                task_logger.create_task(self.upload(image), name='upload_image')
                break

        detecting_cameras = [c for c in self.world.usb_cameras.values() if c.detect]
        if not detecting_cameras:
            await sleep(0.02)
            return

        for camera in detecting_cameras:
            image = camera.images[-1]
            await self.detect(image)

    async def detect(self, image: Image) -> Image:
        if not self.is_connected:
            return
        try:
            result = await self.sio.call('detect', {'image': image.data, 'mac': image.camera_id})
            box_detections = [BoxDetection.parse_obj(d) for d in result.get('box_detections', [])]
            point_detections = [PointDetection.parse_obj(d) for d in result.get('point_detections', [])]
            image.detections = box_detections + point_detections
        except:
            self.log.exception(f'could not detect {image.id}')
        else:
            event.emit(event.Id.NEW_DETECTIONS, image)
            return image

    async def upload(self, image: Image):
        try:
            await self.sio.emit('upload', {'image': image.data, 'mac': image.camera_id})
        except:
            self.log.exception(f'could not upload {image.id}')

    def __str__(self) -> str:
        state = {
            None: 'starting',
            True: 'connected',
            False: 'reconnecting',
        }[self.is_connected]
        return f'{type(self).__name__} ({state})'
