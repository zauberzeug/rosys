import asyncio
import logging
from datetime import datetime, timedelta
from typing import Optional

import rosys
import socketio
import socketio.exceptions

from .. import persistence, task_logger
from .detections import BoxDetection, Detections, PointDetection, SegmentationDetection
from .detector import Autoupload, Detector
from .image import Image
from .uploads import Uploads


class DetectorHardware(Detector):
    '''This detector communicates with a [YOLO detector](https://hub.docker.com/r/zauberzeug/yolov5-detector) via Socket.IO.

    It automatically connects and reconnects, submits and receives detections and sends images that should be uploaded to the [Zauberzeug Learning Loop](https://zauberzeug.com/learning-loop.html).
    '''

    def __init__(self, *, port: int = 8004) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.detector')

        self.sio = socketio.AsyncClient()
        self.is_detecting: bool = False
        self.next_image: Optional[Image] = None
        self.port = port
        self.timeout_count = 0
        self._uploads = Uploads()

        @self.sio.on('disconnect')
        def on_sio_disconnect() -> None:
            self.log.warning(f'sio disconnect on port {port}')

        @self.sio.on('connect_error')
        def on_sio_connect_error(err) -> None:
            self.log.warning(f'sio connect error on {port}: {err}')

        rosys.on_repeat(self.step, 1.0)

    @property
    def uploads(self) -> Uploads:
        return self._uploads

    @property
    def is_connected(self) -> bool:
        return self.sio.connected

    async def step(self) -> None:
        if not self.is_connected:
            self.log.info(f'trying reconnect {self.port}')
            if not await self.connect():
                self.log.exception(f'connection to {self.port} failed; trying again')
                await rosys.sleep(3.0)
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
        if datetime.now() < self.uploads.last_upload + timedelta(minutes=self.uploads.minimal_minutes_between_uploads):
            return

        upload_images = self.uploads.get_queued()
        if upload_images:
            task_logger.create_task(self.upload(upload_images[0]), name='upload_image')
            self.uploads.queue.clear()  # old images should not be uploaded later when the robot is inactive
            self.uploads.last_upload = datetime.now()

    async def upload_priority_queue(self) -> None:
        upload_images = self.uploads.get_priority_queued()
        if upload_images:
            async def upload_priority_images():
                for image in upload_images:
                    await self.upload(image)
            task_logger.create_task(upload_priority_images(), name='upload_priority_images')
            self.uploads.priority_queue.clear()

    async def upload(self, image: Image) -> None:
        try:
            self.log.info(f'uploading to port {self.port}')
            await self.sio.emit('upload', {'image': image.data, 'mac': image.camera_id})
        except:
            self.log.exception(f'could not upload {image.id}')

    async def detect(self, image: Image, autoupload: Autoupload = Autoupload.FILTERED, tags: list[str] = []) -> None:
        '''Runs detections on the image. Afterwards the `image.detections` property is filled.'''
        if not self.is_connected:
            return

        self.next_image = image
        if self.is_detecting:
            return

        while self.next_image is not None and not rosys.is_stopping():
            try:
                image = self.next_image
                if image.is_broken:
                    continue
                self.next_image = None
                self.is_detecting = True
                result: dict = await self.sio.call('detect', {
                    'image': image.data,
                    'mac': image.camera_id,
                    'autoupload': autoupload.value,
                    'tags': tags,
                }, timeout=3)
                if image.is_broken:  # NOTE: image can be marked broken while detection is underway
                    continue
                image.detections = Detections(
                    boxes=[persistence.from_dict(BoxDetection, d) for d in result.get('box_detections', [])],
                    points=[persistence.from_dict(PointDetection, d) for d in result.get('point_detections', [])],
                    segmentations=[
                        persistence.from_dict(SegmentationDetection, d)
                        for d in result.get('segmentation_detections', [])
                    ],
                )
            except socketio.exceptions.TimeoutError:
                self.log.debug(f'detection for {image.id} on {self.port} took too long')
                self.timeout_count += 1
            except asyncio.exceptions.CancelledError:
                self.log.debug(f'task has been cancelled')
            except:
                self.log.exception(f'could not detect {image.id}')
            else:
                self.timeout_count = 0
                self.NEW_DETECTIONS.emit(image)
            finally:
                self.is_detecting = False
                if self.timeout_count > 5:
                    await self.disconnect()

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'
