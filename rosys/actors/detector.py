from datetime import datetime, timedelta
from typing import Optional

import socketio

from .. import event, task_logger
from ..helpers import sleep
from ..world import BoxDetection, Detections, Image, PointDetection
from .actor import Actor


class Detector(Actor):
    interval: float = 1.0

    def __init__(self, port: int = 8004, name: str = 'detector'):
        super().__init__()
        self.sio = socketio.AsyncClient()
        self.is_detecting: bool = False
        self.next_image: Optional[Image] = None
        self.port = port
        self.name = name

    @property
    def is_connected(self):
        return self.sio.connected

    async def step(self):
        if not self.is_connected and not await self.connect():
            await sleep(3.0)
            return

        await self.try_start_one_upload()
        await self.upload_priority_queue()

    async def connect(self) -> bool:
        try:
            url = f'ws://localhost:{self.port}'
            self.log.info(f'connecting to detector at {url}')
            await self.sio.connect(url, socketio_path='/ws/socket.io', wait_timeout=3.0)
            self.log.info('connected successfully')
            return True
        except:
            self.log.exception('connection failed; trying again')
            return False

    async def try_start_one_upload(self):
        if datetime.now() < self.world.upload.last_upload + timedelta(minutes=self.world.upload.minimal_minutes_between_uploads):
            return

        upload_images = self.world.upload.get_queued(self.name)
        if upload_images:
            task_logger.create_task(self.upload(upload_images[0]), name='upload_image')
            self.world.upload.queue.clear()  # old images should not be uploaded later when the robot is inactive
            self.world.upload.last_upload = datetime.now()

    async def upload_priority_queue(self):
        upload_images = self.world.upload.get_priority_queued(self.name)
        if upload_images:
            async def upload_priority_images():
                for image in upload_images:
                    await self.upload(image)
            task_logger.create_task(upload_priority_images(), name='upload_priority_images')
            self.world.upload.priority_queue.clear()

    async def upload(self, image: Image):
        try:
            self.log.info(f'uploading to {self.name}')
            await self.sio.emit('upload', {'image': image.data, 'mac': image.camera_id})
        except:
            self.log.exception(f'could not upload {image.id}')

    async def detect(self, image: Image, submission_criteria: str = 'novel,uncertain') -> None:
        '''Runs detections and adds them to the provided image.

        `submission_criteria` comma seperated string with criteria which must be true before auto-submitting to the Learning Loop
           - `uncertain`: only submit images with detections having a confidence between 0.3 and 0.6
           - `novel`: only submit images which have some novel detections (helps to prevent multiple image uploads from the same scene)
        '''
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
                result = await self.sio.call('detect', {
                    'image': image.data,
                    'mac': image.camera_id,
                    'submission_criteria': submission_criteria
                }, timeout=1)
                box_detections = [BoxDetection.parse_obj(d) for d in result.get('box_detections', [])]
                point_detections = [PointDetection.parse_obj(d) for d in result.get('point_detections', [])]
                image.detections = Detections(boxes=box_detections, points=point_detections)
            except:
                self.log.exception(f'could not detect {image.id}')
            else:
                event.emit(event.Id.NEW_DETECTIONS, image)
            finally:
                self.is_detecting = False

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'
