import asyncio
from datetime import datetime
from typing import Any, Literal

import numpy as np
import socketio
import socketio.exceptions

from .. import rosys
from ..helpers import LazyWorker
from ..persistence import from_dict
from .annotations import Annotations
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
    """This detector communicates with a zauberzeug detector node via SocketIO.

    It automatically connects and reconnects, submits and receives detections and sends images
    that should be uploaded to the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).

    Note: Images must be smaller than ``MAX_IMAGE_SIZE`` bytes (default: 64 MB).
    """
    MAX_IMAGE_SIZE = 64 * 1024 * 1024
    MAX_TIMEOUTS_BEFORE_DISCONNECT = 5

    def __init__(self,
                 *,
                 host: str = 'localhost',
                 port: int = 8004,
                 name: str | None = None,
                 auto_disconnect: bool = True,
                 sio_timeout: int = 5) -> None:
        super().__init__(name=name)

        self.lazy_worker = LazyWorker()
        self.host = host
        self.port = port
        self.auto_disconnect = auto_disconnect

        self.sio_timeout = sio_timeout
        self.timeout_count = 0

        self.sio = socketio.AsyncClient(websocket_extra_options={'max_msg_size': self.MAX_IMAGE_SIZE + 1000})
        self.sio.on('disconnect', lambda: self.log.warning('disconnect on %s:%s', self.host, self.port))
        self.sio.on('connect_error', lambda e: self.log.warning('connect error on %s:%s %s', self.host, self.port, e))

        rosys.on_startup(self.connect)
        rosys.on_repeat(self._ensure_connection, 10.0)

    @property
    def is_connected(self) -> bool:
        return self.sio.connected

    async def _ensure_connection(self) -> None:
        if not self.is_connected:
            self.log.info('trying reconnect %s', self.name)
            if not await self.connect():
                await self.sio.disconnect()
                self.log.error('connection to %s at port %s failed; trying again', self.name, self.port)

    async def connect(self) -> bool:
        try:
            url = f'ws://{self.host}:{self.port}'
            self.log.info('connecting to detector at %s', url)
            await self.sio.connect(url, socketio_path='/ws/socket.io', wait_timeout=3)
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
                     creation_date: datetime | str | None = None,
                     annotations: Annotations | None = None) -> None:

        if not self.is_connected:
            self.log.error('Upload failed: detector is not connected')
            raise DetectorException('detector is not connected')

        if image.byte_size() > self.MAX_IMAGE_SIZE:
            self.log.error('Upload failed: image too large: %s', image.byte_size())
            raise DetectorException('image too large')

        try:
            self.log.info('Upload detections to port %s', self.port)

            metadata: dict[str, Any] = {
                'source': source,
                'tags': tags or [],
                'creation_date': _creation_date_to_isoformat(creation_date),
            }

            if detections := image.get_detections(self.name):
                detections_dict = detections.to_dict()
                metadata.update({
                    'box_detections': _box_detections_to_int(detections_dict['boxes']),
                    'point_detections': detections_dict['points'],
                    'segmentation_detections': detections_dict['segmentations'],
                    'classification_detections': detections_dict['classifications'],
                })

            if annotations is not None:
                annotations_dict = annotations.to_dict()
                metadata.update({
                    'box_annotations': annotations_dict['box_annotations'],
                    'point_annotations': annotations_dict['point_annotations'],
                    'classification_annotation': annotations_dict['classification_annotation'],
                })

            await self.sio.emit('upload', {
                'image': _array_to_dict(image.array),
                'metadata': metadata,
            })

        except Exception as e:
            self.log.error('Upload failed with exception: %s', e)
            raise DetectorException('Upload failed') from e

    async def detect(self,
                     image: Image,
                     *,
                     lazy: bool = True,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None) -> Detections | None:

        if not self.is_connected:
            self.log.error('Detection failed: detector is not connected')
            raise DetectorException('detector is not connected')

        if image.byte_size() > self.MAX_IMAGE_SIZE:
            self.log.error('Detection failed: image too large: %s bytes', image.byte_size())
            raise DetectorException('image too large')

        try:
            detect_call = self._detect(image, autoupload, tags or [], source, creation_date)
            if lazy:
                detections = await self.lazy_worker.run(detect_call)
            else:
                detections = await detect_call
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
                      creation_date: datetime | str | None = None) -> Detections:
        try:
            response = await self.sio.call('detect', {
                'image': _array_to_dict(image.array),
                'camera_id': image.camera_id,
                'tags': tags,
                'source': source,
                'autoupload': autoupload.value,
                'creation_date': _creation_date_to_isoformat(creation_date),
            }, timeout=self.sio_timeout)
            self.timeout_count = 0
        except socketio.exceptions.TimeoutError:
            self.timeout_count += 1
            if self.timeout_count > self.MAX_TIMEOUTS_BEFORE_DISCONNECT and self.auto_disconnect:
                self.log.error('Detection timed out %d times in a row. Disconnecting from detector %s',
                               self.timeout_count, self.name)
                await self.disconnect()
            raise DetectorException('Detection timeout') from None
        except Exception as e:
            raise DetectorException('Detection failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Invalid response from detector')
        try:
            detections = _detections_from_dict(response)
        except Exception as e:
            raise DetectorException('Failed to parse detections') from e

        return detections

    async def batch_detect(self,
                           images: list[Image],
                           *,
                           lazy: bool = True,
                           autoupload: Autoupload = Autoupload.FILTERED,
                           tags: list[str] | None = None,
                           source: str | None = None,
                           creation_date: datetime | str | None = None) -> list[Detections] | None:

        if not self.is_connected:
            self.log.error('Detection failed: detector is not connected')
            raise DetectorException('detector is not connected')

        total_size = sum(image.byte_size() for image in images)
        if total_size > self.MAX_IMAGE_SIZE:
            self.log.error('Detection failed: total image size too large: %s bytes', total_size)
            raise DetectorException('total image size too large')

        try:
            detect_call = self._batch_detect(images, autoupload, tags or [], source, creation_date)
            if lazy:
                batch_detections = await self.lazy_worker.run(detect_call)
            else:
                batch_detections = await detect_call

            if batch_detections is not None:
                if len(images) != len(batch_detections):
                    self.log.error('Batch detection failed: number of detections does not match number of images')
                    raise DetectorException('number of detections does not match number of images')

                for image, detections in zip(images, batch_detections, strict=True):
                    image.set_detections(self.name, detections)
                    self.NEW_DETECTIONS.emit(image)
            return batch_detections
        except asyncio.exceptions.CancelledError:
            raise DetectorException('Detection cancelled') from None

    async def _batch_detect(self,
                            images: list[Image],
                            autoupload: Autoupload,
                            tags: list[str],
                            source: str | None = None,
                            creation_date: datetime | str | None = None) -> list[Detections]:

        if len(images) == 0:
            return []

        try:
            # List of image data dictionaries, each with the same structure as the `image` entry in the `detect` endpoint
            images_dict = [_array_to_dict(image.array) for image in images]
            response = await self.sio.call('batch_detect', {
                'images': images_dict,
                'camera_id': images[0].camera_id if images else None,
                'tags': tags,
                'source': source,
                'autoupload': autoupload.value,
                'creation_date': _creation_date_to_isoformat(creation_date),
            }, timeout=self.sio_timeout)
            self.timeout_count = 0
        except socketio.exceptions.TimeoutError:
            self.timeout_count += 1
            if self.timeout_count > self.MAX_TIMEOUTS_BEFORE_DISCONNECT and self.auto_disconnect:
                self.log.error('Detection timed out %d times in a row. Disconnecting from detector %s',
                               self.timeout_count, self.name)
                await self.disconnect()
            raise DetectorException('Detection timeout') from None
        except Exception as e:
            raise DetectorException('Detection failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Invalid response from detector')
        try:
            all_detections = [_detections_from_dict(d) for d in response.get('items', [])]
        except Exception as e:
            raise DetectorException('Failed to parse detections') from e

        return all_detections

    def __str__(self) -> str:
        return f'{type(self).__name__} ({"connected" if self.is_connected else "disconnected"})'

    async def fetch_detector_info(self) -> DetectorInfo:
        if not self.is_connected:
            raise DetectorException('detector is not connected')

        try:
            response = await self.sio.call('about', timeout=self.sio_timeout)
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
            response = await self.sio.call('get_model_version', timeout=self.sio_timeout)
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
            response = await self.sio.call('set_model_version_mode', version, timeout=self.sio_timeout)
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

    async def fetch_outbox_mode(self) -> Literal['continuous_upload', 'stopped']:
        try:
            response = await self.sio.call('get_outbox_mode', timeout=self.sio_timeout)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timeout') from None
        except Exception as e:
            self.log.error('Communication failed for detector %s: %s', self.name, e)
            raise DetectorException('Communication failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Failed to get outbox mode')

        mode = response.get('outbox_mode')
        if mode not in ['continuous_upload', 'stopped']:
            raise DetectorException(f'Invalid outbox mode received: {mode}')

        return mode

    async def set_outbox_mode(self, mode: Literal['continuous_upload', 'stopped']) -> None:
        try:
            response = await self.sio.call('set_outbox_mode', mode, timeout=self.sio_timeout)
        except socketio.exceptions.TimeoutError:
            self.log.error('Communication timeout for detector %s', self.name)
            raise DetectorException('Communication timeout') from None
        except Exception as e:
            self.log.error('Communication failed for detector %s: %s', self.name, e)
            raise DetectorException('Communication failed') from e

        if not isinstance(response, dict):
            raise DetectorException('Failed to set outbox mode')

        if response.get('status') != 'OK':
            error_message = response.get('error', 'unknown error')
            raise DetectorException(f'Failed to set outbox mode: {error_message}')

    async def soft_reload(self) -> None:
        """Trigger a soft reload of the detector.

        :raises DetectorException: if the communication fails.
        """
        try:
            await self.sio.call('soft_reload', timeout=self.sio_timeout)
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


def _detections_from_dict(data: dict[str, Any]) -> Detections:
    return Detections(
        boxes=[from_dict(BoxDetection, d) for d in data.get('box_detections', [])],
        points=[from_dict(PointDetection, d) for d in data.get('point_detections', [])],
        segmentations=[from_dict(SegmentationDetection, d) for d in data.get('segmentation_detections', [])],
        classifications=[from_dict(ClassificationDetection, d) for d in data.get('classification_detections', [])],
    )


def _array_to_dict(array: np.ndarray) -> dict[str, bytes | str | tuple[int, ...]]:
    return {
        'bytes': array.tobytes(order='C'),
        'dtype': str(array.dtype),
        'shape': array.shape,
    }
