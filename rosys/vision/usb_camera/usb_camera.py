import logging
from typing import Any, Optional, Self

import cv2

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image, ImageSize
from ..image_processing import process_jpeg_image, process_ndarray_image, to_bytes
from ..image_rotation import ImageRotation
from .usb_device import UsbDevice


class UsbCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: Optional[str] = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 auto_exposure: bool = True,
                 exposure: bool = False,
                 width: int = 800,
                 height: int = 600,
                 fps: int = 10,
                 **kwargs) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         streaming=streaming,
                         **kwargs)
        self._pending_operations = 0
        self.device: Optional[UsbDevice] = None
        self.detect: bool = False
        self.color: Optional[str] = None

        self._register_parameter('auto_exposure', self.get_exposure, self.set_exposure, auto_exposure)
        self._register_parameter('exposure', self.get_exposure, self.set_exposure, exposure)
        self._register_parameter('width', self.get_width, self.set_width, width)
        self._register_parameter('height', self.get_height, self.set_height, height)
        self._register_parameter('fps', self.get_fps, self.set_fps, fps)

    def to_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
        } | {
            name: param.value for name, param in self._parameters.items()
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**data)

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        logging.info(f'Connecting camera {self.id}...')
        if self.is_connected:
            return

        device = UsbDevice.from_uid(self.id)
        if device is None:
            logging.warning(f'Connecting to {self.id} failed!')
            return

        self.device = device

        logging.info(f'camera {self.id}: connected')

        self._apply_all_parameters()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None
        await rosys.run.io_bound(self.device.capture.release)
        self.device = None
        logging.info(f'camera {self.id}: disconnected')

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None

        assert self.device is not None

        capture_success, captured_image = await rosys.run.io_bound(self.device.capture.read)
        image_is_MJPG = 'MJPG' in self.device.video_formats

        if not capture_success:
            await self.disconnect()
            return

        if captured_image is None:
            return

        if image_is_MJPG:
            bytes_ = await rosys.run.io_bound(to_bytes, captured_image)
            if self.crop or self.rotation != ImageRotation.NONE:
                bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, self.rotation, self.crop)
        else:
            bytes_ = await rosys.run.cpu_bound(process_ndarray_image, captured_image, self.rotation, self.crop)

        image_size = ImageSize(width=captured_image.shape[1], height=captured_image.shape[0])
        final_image_resolution = self._resolution_after_transform(image_size)

        image = Image(time=rosys.time(), camera_id=self.id, size=final_image_resolution, data=bytes_)
        self._add_image(image)

    def set_auto_exposure(self, auto: bool) -> None:
        assert self.device is not None

        device = self.device

        if device.has_manual_exposure:
            is_auto_exposure = self.get_auto_exposure()
            if auto and not is_auto_exposure:
                # self.log.info(f'activating auto-exposure for {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                self.set_exposure(self._parameters['exposure'].value)

    def set_exposure(self, value: float) -> None:
        assert self.device is not None

        device = self.device
        assert device.capture is not None

        if device.has_manual_exposure:
            is_auto_exposure = self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
                if value != exposure:
                    device.capture.set(cv2.CAP_PROP_EXPOSURE, int(value * device.exposure_max))

    def get_auto_exposure(self) -> Optional[bool]:
        assert self.device is not None
        device = self.device
        return device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    def get_exposure(self) -> Optional[float]:
        assert self.device is not None

        device = self.device
        if not device.has_manual_exposure:
            return None
        return device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max

    def set_width(self, width: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def get_width(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    def set_height(self, height: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_height(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_fps(self, fps: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FPS, fps)

    def get_fps(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FPS))
