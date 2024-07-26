import asyncio
from datetime import datetime, timedelta
from typing import Any

import socketio
import socketio.exceptions

from .. import persistence, rosys
from ..helpers import LazyWorker
from .detections import BoxDetection, Detections, PointDetection, SegmentationDetection
from .detector import Autoupload, Detector
from .image import Image


class DetectorHardware(Detector):
    """This detector communicates with a [YOLO detector](https://hub.docker.com/r/zauberzeug/yolov5-detector) via Socket.IO.

    It automatically connects and reconnects, submits and receives detections and sends images that should be uploaded to the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).
    """

    def __init__(self, *, port: int = 8004, name: str | None = None) -> None:
        super().__init__(name=name)

        self.sio = socketio.AsyncClient()
        self.lazy_worker = LazyWorker()
        self.port = port
        self.timeout_count = 0

        @self.sio.on('disconnect')
        def on_sio_disconnect() -> None:
            self.log.warning('sio disconnect on port %s', port)

        @self.sio.on('connect_error')
        def on_sio_connect_error(err) -> None:
            self.log.warning('sio connect error on %s: %s', port, err)

        rosys.on_repeat(self.step, 1.0)

    @property
    def is_connected(self) -> bool:
        return self.sio.connected

    async def step(self) -> None:
        if not self.is_connected:
            self.log.info('trying reconnect %s', self.name)
            if not await self.connect():
                self.log.error('connection to %s at port %s failed; trying again', self.name, self.port)
                await rosys.sleep(3.0)
                return

        await self.try_start_one_upload()
        await self.upload_priority_queue()

    async def connect(self) -> bool:
        try:
            url = f'ws://localhost:{self.port}'
            self.log.info('connecting to detector at %s', url)
            await self.sio.connect(url, socketio_path='/ws/socket.io', wait_timeout=3.0)
            self.log.info('connected successfully')
            return True
        except Exception:
            return False

    async def disconnect(self) -> None:
        await self.sio.disconnect()

    async def try_start_one_upload(self) -> None:
        if datetime.now() < self.uploads.last_upload + timedelta(minutes=self.uploads.minimal_minutes_between_uploads):
            return

        upload_images = self.uploads.get_queued()
        if upload_images:
            rosys.background_tasks.create(self.upload(upload_images[0]), name='upload_image')
            self.uploads.queue.clear()  # old images should not be uploaded later when the robot is inactive
            self.uploads.last_upload = datetime.now()

    async def upload_priority_queue(self) -> None:
        upload_images = self.uploads.get_priority_queued()
        if upload_images:
            async def upload_priority_images():
                for image in upload_images:
                    await self.upload(image)
            rosys.background_tasks.create(upload_priority_images(), name='upload_priority_images')
            self.uploads.priority_queue.clear()

    async def upload(self, image: Image, *, tags: list[str] | None = None) -> None:
        tags = tags or []
        try:
            self.log.info('Upload detections to port %s', self.port)
            data_dict: dict[str, Any] = {'image': image.data, 'mac': image.camera_id}
            detections = image.get_detections(self.name)
            if detections is not None:
                detections_dict = detections.to_dict()
                detections_dict['box_detections'] = _box_detections_to_int(detections_dict.pop('boxes'))
                detections_dict['point_detections'] = detections_dict.pop('points')
                detections_dict['segmentation_detections'] = detections_dict.pop('segmentations')
                data_dict['detections'] = detections_dict
            data_dict['tags'] = list(image.tags.union(tags))
            await self.sio.emit('upload', data_dict)

        except Exception:
            self.log.exception('could not upload %s', image.id)

    async def detect(self,
                     image: Image,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     ) -> Detections | None:
        tags = tags or []
        return await self.lazy_worker.run(self._detect(image, autoupload, tags))

    async def _detect(self, image: Image, autoupload: Autoupload, tags: list[str]) -> Detections | None:
        if image.is_broken:
            return None
        try:
            result: dict = await self.sio.call('detect', {
                'image': image.data,
                'mac': image.camera_id,
                'autoupload': autoupload.value,
                'tags': tags,
            }, timeout=3)
            if image.is_broken:  # NOTE: image can be marked broken while detection is underway
                return None
            detections = Detections(
                boxes=[persistence.from_dict(BoxDetection, d) for d in result.get('box_detections', [])],
                points=[persistence.from_dict(PointDetection, d) for d in result.get('point_detections', [])],
                segmentations=[
                    persistence.from_dict(SegmentationDetection, d)
                    for d in result.get('segmentation_detections', [])
                ],
            )
            image.set_detections(self.name, detections)
            self.timeout_count = 0
            if image.is_broken:
                return None
            self.NEW_DETECTIONS.emit(image)
            return detections
        except socketio.exceptions.TimeoutError:
            self.log.debug('detection for %s on %s took too long', image.id, self.port)
            self.timeout_count += 1
        except asyncio.exceptions.CancelledError:
            self.log.debug('task has been cancelled')
        except Exception:
            self.log.exception('could not detect %s', image.id)
        finally:
            if self.timeout_count > 5:
                await self.disconnect()
        return None

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'


def _box_detections_to_int(detections: list[dict]) -> list[dict]:
    for detection in detections:
        detection['x'] = int(detection['x'])
        detection['y'] = int(detection['y'])
        detection['width'] = int(detection['width'])
        detection['height'] = int(detection['height'])
    return detections
