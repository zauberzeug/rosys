import socketio
from typing import Optional
from datetime import datetime, timedelta
from .. import event, sleep, task_logger
from ..world import BoxDetection, Image, PointDetection
from .actor import Actor


class Detector(Actor):
    interval: float = 1.0

    def __init__(self):
        super().__init__()
        self.sio = socketio.AsyncClient()
        self.is_detecting: bool = False
        self.next_image: Optional[Image] = None

    @property
    def is_connected(self):
        return self.sio.connected

    async def step(self):
        if not self.is_connected and not await self.connect():
            await sleep(3.0)
            return

        await self.try_start_one_upload()

    async def connect(self) -> bool:
        try:
            self.log.info('connecting to detector')
            await self.sio.connect('ws://localhost:8004', socketio_path='/ws/socket.io', wait_timeout=3.0)
            self.log.info('connected successfully')
            return True
        except:
            self.log.exception('connection failed; trying again')
            return False

    async def try_start_one_upload(self):
        if datetime.now() < self.world.upload.last_upload + timedelta(minutes=self.world.upload.minimal_minutes_between_uploads):
            return

        upload_images = [image for image in self.world.upload.queue if image.data]
        if not upload_images:
            return

        self.world.upload.last_upload = datetime.now()
        self.world.upload.queue.clear()  # old images should not be uploaded later when the robot is inactive
        task_logger.create_task(self.upload(upload_images[0]), name='upload_image')

    async def upload(self, image: Image):
        try:
            await self.sio.emit('upload', {'image': image.data, 'mac': image.camera_id})
        except:
            self.log.exception(f'could not upload {image.id}')

    async def detect(self, image: Image):
        if not self.is_connected:
            return

        self.next_image = image
        if self.is_detecting:
            return

        while self.next_image is not None:
            try:
                image = self.next_image
                self.next_image = None
                self.is_detecting = True
                result = await self.sio.call('detect', {'image': image.data, 'mac': image.camera_id}, timeout=1)
                box_detections = [BoxDetection.parse_obj(d) for d in result.get('box_detections', [])]
                point_detections = [PointDetection.parse_obj(d) for d in result.get('point_detections', [])]
                image.detections = box_detections + point_detections
            except:
                self.log.exception(f'could not detect {image.id}')
            else:
                event.emit(event.Id.NEW_DETECTIONS, image)
            finally:
                self.is_detecting = False

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'
