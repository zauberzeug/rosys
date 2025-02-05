import logging
from typing import Any

import numpy as np

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image, process_ndarray_image
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
        await self.device.release_capture()
        self.device = None
        logging.info('camera %s: disconnected', self.id)

    async def _handle_new_image_data(self, image_data: np.ndarray | bytes, timestamp: float) -> None:
        if not self.is_connected:
            return None

        assert self.device is not None

        bytes_: bytes | None
        if isinstance(image_data, np.ndarray):
            bytes_ = await rosys.run.cpu_bound(process_ndarray_image, image_data, self.rotation, self.crop)
        else:
            bytes_ = image_data
            if self.crop or self.rotation != ImageRotation.NONE:
                bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, self.rotation, self.crop)
        if bytes_ is None:
            return

        final_image_resolution = get_image_size_from_bytes(bytes_)

        image = Image(time=timestamp, camera_id=self.id, size=final_image_resolution, data=bytes_)
        self._add_image(image)

    def set_auto_exposure(self, auto: bool) -> None:
        assert self.device is not None
        self.device.set_auto_exposure(auto)
        if not auto:
            manual_exposure = self._parameters['exposure'].value
            assert manual_exposure is not None
            self.device.set_exposure(manual_exposure)

    def set_exposure(self, value: float) -> None:
        assert self.device is not None
        self.device.set_exposure(value)

    def get_auto_exposure(self) -> bool | None:
        assert self.device is not None
        return self.device.get_auto_exposure()

    def get_exposure(self) -> float | None:
        assert self.device is not None
        return self.device.get_exposure()

    def set_width(self, width: int) -> None:
        assert self.device is not None
        self.device.set_width(width)

    def get_width(self) -> int:
        assert self.device is not None
        return self.device.get_width()

    def set_height(self, height: int) -> None:
        assert self.device is not None
        self.device.set_height(height)

    def get_height(self) -> int:
        assert self.device is not None
        return self.device.get_height()

    def set_fps(self, fps: int) -> None:
        assert self.device is not None
        self.device.set_fps(fps)

    def get_fps(self) -> int:
        assert self.device is not None
        return self.device.get_fps()
