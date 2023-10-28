import logging
from typing import Any, Optional

import cv2

import rosys

from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image, ImageSize
from ..image_processing import process_jpeg_image, process_ndarray_image, to_bytes
from ..image_rotation import ImageRotation
from .usb_device import UsbDevice


class UsbCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,
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

        self._register_parameter(name='auto_exposure', setter=self.set_exposure,
                                 getter=self.get_exposure, default_value=auto_exposure)
        self._register_parameter(name='exposure', setter=self.set_exposure,
                                 getter=self.get_exposure, default_value=exposure)
        self._register_parameter(name='width', setter=self.set_width, getter=self.get_width, default_value=width)
        self._register_parameter(name='height', setter=self.set_height, getter=self.get_height, default_value=height)
        self._register_parameter(name='fps', setter=self.set_fps, getter=self.get_fps, default_value=fps)

    def to_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
            'parameters': {name: param.value for name, param in self._parameters.items()},
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'UsbCamera':

        return cls(
            id=data['id'],
            name=data['name'],
            connect_after_init=data['connect_after_init'],
            streaming=data['streaming'],
            auto_exposure=data.get('parameters', {}).get('auto_exposure', None),
            exposure=data.get('parameters', {}).get('exposure', None),
            width=data.get('parameters', {}).get('width', None),
            height=data.get('parameters', {}).get('height', None),
            fps=data.get('parameters', {}).get('fps', None)
        )

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        logging.info(f'Connecting camera {self.id}...')
        async with self._device_connection():
            if self.is_connected:
                return
            self._pending_operations += 1

        device = await UsbDevice.from_uid(self.id)

        if device is None:
            logging.warning(f'Connecting to {self.id} failed!')
            return

        self.device = device

        async with self._device_connection():
            self._pending_operations -= 1
            self.device_connection_lock.notify_all()

        logging.info(f'camera {self.id}: connected')

        await self._apply_all_parameters()
        await self._update_parameter_values()

    async def disconnect(self) -> None:
        async with self._device_connection():
            if not self.is_connected:
                return
            logging.info(f'camera {self.id}: disconnect initialized...')
            while self._pending_operations > 0:
                logging.info(f'camera {self.id}: waiting for pending operations to finish...')
                await self.device_connection_lock.wait()

            assert self.device is not None
            await rosys.run.io_bound(self.device.capture.release)
            self.device = None
            logging.info(f'camera {self.id}: disconnected')

    async def capture_image(self) -> None:
        async with self._device_connection():
            if not self.is_connected:
                return None

            assert self.device is not None

            capture_success, captured_image = self.device.capture.read()
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

        final_image_resolution = self._resolution_after_transform(
            ImageSize(width=captured_image.shape[1], height=captured_image.shape[0]))

        image = Image(time=rosys.time(), camera_id=self.id, size=final_image_resolution, data=bytes_)
        self._add_image(image)

    async def set_auto_exposure(self, auto: bool) -> None:
        assert self.device is not None

        device = self.device

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if auto and not is_auto_exposure:
                # self.log.info(f'activating auto-exposure for {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                await self.set_exposure(self._parameters['exposure'].value)

    async def set_exposure(self, value: float) -> None:
        assert self.device is not None

        device = self.device
        assert device.capture is not None

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
                if value != exposure:
                    device.capture.set(cv2.CAP_PROP_EXPOSURE, int(value * device.exposure_max))

    async def get_auto_exposure(self) -> Optional[bool]:
        assert self.device is not None
        device = self.device
        return device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    async def get_exposure(self) -> Optional[float]:
        assert self.device is not None

        device = self.device
        if not device.has_manual_exposure:
            return None
        return device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max

    async def set_width(self, width: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    async def get_width(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    async def set_height(self, height: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    async def get_height(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    async def set_fps(self, fps: int) -> None:
        assert self.device is not None
        device = self.device
        device.capture.set(cv2.CAP_PROP_FPS, fps)

    async def get_fps(self) -> int:
        assert self.device is not None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FPS))
