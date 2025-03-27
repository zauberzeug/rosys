import asyncio
from datetime import datetime
from typing import Any, Literal

import socketio
import socketio.exceptions

from .. import persistence, rosys
from ..helpers import LazyWorker
from .detections import (
    BoxDetection,
    Category,
    ClassificationDetection,
    Detections,
    PointDetection,
    SegmentationDetection,
)
from .detector import Autoupload, Detector, DetectorException, DetectorInfo, ModelVersioningInfo
from .image import Image


class DetectorHardware(Detector):
    """This detector communicates with a [YOLO detector](https://hub.docker.com/r/zauberzeug/yolov5-detector) via Socket.IO.

    It automatically connects and reconnects, submits and receives detections and sends images
    that should be uploaded to the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).

    Note: Images must be smaller than ``MAX_IMAGE_SIZE`` bytes (default: 10 MB).
    """
    MAX_IMAGE_SIZE = 10 * 1024 * 1024

    def __init__(self,
                 *,
                 port: int = 8004,
                 name: str | None = None,
                 auto_disconnect: bool = True) -> None:
        super().__init__(name=name)

        websocket_options = {
            'max_msg_size': self.MAX_IMAGE_SIZE + 1000,
        }
        self.sio = socketio.AsyncClient(websocket_extra_options=websocket_options)
        self.lazy_worker = LazyWorker()
        self.port = port
        self.auto_disconnect = auto_disconnect
        self.timeout_count = 0

        @self.sio.on('disconnect')
        def on_sio_disconnect() -> None:
            self.log.warning('sio disconnect on port %s', port)

        @self.sio.on('connect_error')
        def on_sio_connect_error(err) -> None:
            self.log.warning('sio connect error on %s: %s', port, err)

        rosys.on_repeat(self._ensure_connection, 10.0)

    @property
    def is_connected(self) -> bool:
        return self.sio.connected

    async def _ensure_connection(self) -> None:
        if not self.is_connected:
            self.log.info('trying reconnect %s', self.name)
            if not await self.connect():
                self.log.error('connection to %s at port %s failed; trying again', self.name, self.port)

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

    async def upload(self,
                     image: Image,
                     *,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None
                     ) -> None:
        if not self.is_connected:
            self.log.error('Upload failed: detector is not connected')
            raise DetectorException('detector is not connected')

        if not image.data:
            self.log.error('Upload failed: image data is empty')
            raise DetectorException('image data is empty')

        if len(image.data) > self.MAX_IMAGE_SIZE:
            self.log.error('Upload failed: image too large: %s', len(image.data))
            raise DetectorException('image too large')

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
            data_dict['source'] = source
            data_dict['tags'] = tags
            data_dict['creation_date'] = _creation_date_to_isoformat(creation_date)
            await self.sio.emit('upload', data_dict)

        except Exception as e:
            self.log.error('Upload failed with exception: %s', e)
            raise DetectorException('Upload failed') from e

    async def detect(self,
                     image: Image,
                     *,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> Detections | None:
        if not self.is_connected:
            self.log.error('Detection failed: detector is not connected')
            raise DetectorException('detector is not connected')

        if image.is_broken:
            self.log.error('Detection failed: image is broken')
            raise DetectorException('image is broken')

        assert len(image.data or []) < self.MAX_IMAGE_SIZE, f'image too large: {len(image.data or [])}'
        tags = tags or []
        try:
            detections = await self.lazy_worker.run(self._detect(image, autoupload, tags, source, creation_date))
            if detections is not None:
                image.set_detections(self.name, detections)
                self.NEW_DETECTIONS.emit(image)
            return detections
        except asyncio.exceptions.CancelledError:
            raise DetectorException('Detection cancelled') from None

    async def _detect(self,
                      image: Image,
                      autoupload: Autoupload,
                      tags: list[str],
                      source: str | None = None,
                      creation_date: datetime | str | None = None,
                      ) -> Detections:
        try:
            response = await self.sio.call('detect', {
                'image': image.data,
                'mac': image.camera_id,
                'autoupload': autoupload.value,
                'tags': tags,
                'source': source,
                'creation_date': _creation_date_to_isoformat(creation_date),
            }, timeout=3)
        except socketio.exceptions.TimeoutError:
            self.timeout_count += 1
            if self.timeout_count > 5 and self.auto_disconnect:
                self.log.error('Detection timed out 5 times in a row. Disconnecting from detector %s', self.name)
                await self.disconnect()
            raise DetectorException('Detection timeout') from None
        except Exception as e:
            raise DetectorException('Detection failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Invalid response from detector')
        try:
            detections = Detections(
                boxes=[persistence.from_dict(BoxDetection, d) for d in response.get('box_detections', [])],
                points=[persistence.from_dict(PointDetection, d) for d in response.get('point_detections', [])],
                segmentations=[persistence.from_dict(SegmentationDetection, d)
                               for d in response.get('segmentation_detections', [])],
                classifications=[persistence.from_dict(ClassificationDetection, d)
                                 for d in response.get('classification_detections', [])],
            )
        except Exception as e:
            raise DetectorException('Failed to parse detections') from e

        self.timeout_count = 0
        if image.is_broken:  # NOTE: image can be marked broken while detection is underway
            raise DetectorException('Image is broken')

        return detections

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'

    async def fetch_detector_info(self) -> DetectorInfo:
        if not self.is_connected:
            raise DetectorException('detector is not connected')

        try:
            response = await self.sio.call('about', timeout=5)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timeout') from None
        except Exception as e:
            raise DetectorException('Communication failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Invalid response from detector')

        try:
            model_info_dict = response.get('model_info') or {}
            return DetectorInfo(
                operation_mode=response['operation_mode'],
                state=response.get('state'),
                organization=model_info_dict.get('organization'),
                project=model_info_dict.get('project'),
                current_version=model_info_dict.get('version'),
                categories=[
                    Category(uuid=c['id'],
                             name=c['name'],
                             color=c.get('color'),
                             category_type=c.get('type'))
                    for c in model_info_dict.get('categories', [])
                ],
                resolution=model_info_dict.get('resolution'),
                target_version=response.get('target_model'),
                version_control=response['version_control'],
            )
        except Exception as e:
            raise DetectorException('Failed to parse detector info') from e

    async def fetch_model_version_info(self) -> ModelVersioningInfo:
        try:
            response = await self.sio.call('get_model_version', timeout=5)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timed out') from None
        except Exception as e:
            raise DetectorException('Communication failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Invalid response from detector')

        try:
            return ModelVersioningInfo(
                current_version=response['current_version'],
                target_version=response['target_version'],
                loop_version=response['loop_version'],
                local_versions=response['local_versions'],
                version_control=response['version_control'],
            )
        except Exception as e:
            raise DetectorException('Failed to parse model versioning info') from e

    async def set_model_version(self, version: Literal['follow_loop', 'pause'] | str) -> None:
        if version not in ['follow_loop', 'pause'] and not version.replace('.', '').isdigit():
            raise DetectorException(f'invalid version control mode: {version} '
                                    f'(allowed: "follow_loop", "pause" or a version number like "1.2")')

        try:
            response = await self.sio.call('set_model_version_mode', version, timeout=5)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timeout') from None
        except Exception as e:
            self.log.error('Communication failed for detector %s: %s', self.name, e)
            raise DetectorException('Communication failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Failed to set model version mode')

        if response.get('status') != 'OK':
            error_message = response.get('error', 'unknown error')
            raise DetectorException(f'Failed to set model version mode: {error_message}')

    async def soft_reload(self) -> None:
        """Trigger a soft reload of the detector.

        :raises DetectorException: if the communication fails.
        """
        try:
            await self.sio.call('soft_reload', timeout=5)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timeout') from None
        except Exception as e:
            self.log.error('Communication failed for detector %s: %s', self.name, e)
            raise DetectorException('Communication failed') from e


def _box_detections_to_int(detections: list[dict]) -> list[dict]:
    for detection in detections:
        detection['x'] = int(detection['x'])
        detection['y'] = int(detection['y'])
        detection['width'] = int(detection['width'])
        detection['height'] = int(detection['height'])
    return detections


def _creation_date_to_isoformat(creation_date: datetime | str | None) -> str | None:
    if isinstance(creation_date, str):
        try:
            datetime.fromisoformat(creation_date)  # validate format
        except ValueError as e:
            raise ValueError(f'creation_date string "{creation_date}" is not in valid ISO format') from e
        return creation_date
    elif isinstance(creation_date, datetime):
        return creation_date.isoformat()
    return None
