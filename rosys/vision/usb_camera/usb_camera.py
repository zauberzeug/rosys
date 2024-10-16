import logging
from typing import Any

import cv2
import numpy as np
from typing_extensions import Self

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image, ImageSize
from ..image_processing import process_jpeg_image, process_ndarray_image
from ..image_rotation import ImageRotation
from .usb_device import UsbDevice


class UsbCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 auto_exposure: bool = True,
                 exposure: bool = False,
                 width: int = 800,
                 height: int = 600,
                 fps: int = 10,
                 **kwargs) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         **kwargs)
        self._pending_operations = 0
        self.device: UsbDevice | None = None
        self.detect: bool = False
        self.color: str | None = None

        self._register_parameter('auto_exposure', self.get_exposure, self.set_exposure, auto_exposure)
        self._register_parameter('exposure', self.get_exposure, self.set_exposure, exposure)
        self._register_parameter('width', self.get_width, self.set_width, width)
        self._register_parameter('height', self.get_height, self.set_height, height)
        self._register_parameter('fps', self.get_fps, self.set_fps, fps)

    def to_dict(self) -> dict[str, Any]:
        return super().to_dict() | {
            name: param.value for name, param in self._parameters.items()
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**data)

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if self.is_connected:
            return

        device = UsbDevice.from_uid(self.id, self._handle_new_image_data)
        if device is None:
            logging.warning('Connecting camera %s: failed', self.id)
            return

        self.device = device
        logging.info('Connecting camera %s: succeeded', self.id)

        await self._apply_all_parameters()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None
        await rosys.run.io_bound(self.device.capture.release)
        self.device = None
        logging.info('camera %s: disconnected', self.id)

    async def _handle_new_image_data(self, image_array: np.ndarray) -> None:
        if not self.is_connected:
            return None

        assert self.device is not None

        image_is_MJPG = 'MJPG' in self.device.video_formats

        def to_bytes(image: list[np.ndarray]) -> bytes:
            return image[0].tobytes()

        if image_is_MJPG:
            bytes_ = await rosys.run.io_bound(to_bytes, image_array)
            if self.crop or self.rotation != ImageRotation.NONE:
                bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, self.rotation, self.crop)
        else:
            bytes_ = await rosys.run.cpu_bound(process_ndarray_image, image_array, self.rotation, self.crop)
        if bytes_ is None:
            return

        image_size = ImageSize(width=image_array.shape[1], height=image_array.shape[0])
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

    def get_auto_exposure(self) -> bool | None:
        assert self.device is not None
        device = self.device
        return device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    def get_exposure(self) -> float | None:
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
