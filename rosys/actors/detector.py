from datetime import datetime, timedelta
from typing import List, Optional

import socketio
import socketio.exceptions
from aenum import Enum, auto

from .. import event, task_logger
from ..helpers import sleep
from ..world import BoxDetection, Detections, Image, PointDetection, SegmentationDetection
from .actor import Actor


class Autoupload(Enum, init='value __doc__'):
    '''Configures the auto-submitting of images to the Learning Loop'''

    def _generate_next_value_(name: str, start, count, last_values):
        '''uses enum name as value when calling auto()'''
        return name.lower()

    FILTERED = auto(), 'only submit images with novel detections and in an uncertainty range (this is the default)'
    DISABLED = auto(), 'no auto-submitting'
    ALL = auto(), 'submit all images which are run through the detector'


class Detector(Actor):
    interval: float = 1.0

    def __init__(self, port: int = 8004, name: str = 'detector'):
        super().__init__()
        self.sio = socketio.AsyncClient()
        self.is_detecting: bool = False
        self.next_image: Optional[Image] = None
        self.port = port
        self.name = name
        self.timeout_count = 0

        @self.sio.on('disconnect')
        def on_sio_disconnect():
            self.log.warning(f'sio disconnect on port {port}')

        @self.sio.on('connect_error')
        def on_sio_connect_error(err):
            self.log.warning(f'sio connect error on {port}: {err}')

    @property
    def is_connected(self):
        return self.sio.connected

    async def step(self):
        if not self.is_connected:
            self.log.info(f'trying reconnect {self.port}')
            if not await self.connect():
                self.log.exception(f'connection to {self.port} failed; trying again')
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
            return False

    async def disconnect(self) -> None:
        await self.sio.disconnect()

    async def try_start_one_upload(self) -> None:
        if datetime.now() < self.world.upload.last_upload + timedelta(minutes=self.world.upload.minimal_minutes_between_uploads):
            return

        upload_images = self.world.upload.get_queued(self.name)
        if upload_images:
            task_logger.create_task(self.upload(upload_images[0]), name='upload_image')
            self.world.upload.queue.clear()  # old images should not be uploaded later when the robot is inactive
            self.world.upload.last_upload = datetime.now()

    async def upload_priority_queue(self) -> None:
        upload_images = self.world.upload.get_priority_queued(self.name)
        if upload_images:
            async def upload_priority_images():
                for image in upload_images:
                    await self.upload(image)
            task_logger.create_task(upload_priority_images(), name='upload_priority_images')
            self.world.upload.priority_queue.clear()

    async def upload(self, image: Image) -> None:
        try:
            self.log.info(f'uploading to {self.name}')
            await self.sio.emit('upload', {'image': image.data, 'mac': image.camera_id})
        except:
            self.log.exception(f'could not upload {image.id}')

    async def detect(self, image: Image, autoupload: Autoupload = Autoupload.FILTERED, tags: List[str] = []) -> None:
        '''Runs detections on the image. Afterwards the `image.detections` property is filled.'''
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
                    'autoupload': autoupload.value,
                    'tags': tags,
                }, timeout=3)
                image.detections = Detections(
                    boxes=[BoxDetection.parse_obj(d) for d in result.get('box_detections', [])],
                    points=[PointDetection.parse_obj(d) for d in result.get('point_detections', [])],
                    segmentations=[
                        SegmentationDetection.from_dict(d) for d in result.get('segmentation_detections', [])
                    ],
                )
            except socketio.exceptions.TimeoutError:
                self.log.exception(f'detection for {image.id} on {self.port} took too long')
                self.timeout_count += 1
            except:
                self.log.exception(f'could not detect {image.id}')
            else:
                self.timeout_count = 0
                event.emit(event.Id.NEW_DETECTIONS, image)
            finally:
                self.is_detecting = False
                if self.timeout_count > 5:
                    await self.disconnect()

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'
