from typing import Any, Optional

import cv2

import rosys

from ..camera import Camera, ConfigurableCameraMixin, TransformableCameraMixin
from ..image import Image, ImageSize
from ..image_processing import process_jpeg_image, process_ndarray_image, to_bytes
from ..image_rotation import ImageRotation
from .usb_device import UsbDevice


class UsbCamera(ConfigurableCameraMixin, TransformableCameraMixin, Camera):
    device: Optional[UsbDevice]
    detect: bool
    color: Optional[str]

    def __init__(self, id, name=None, connect_after_init=True, streaming=True,
                 auto_exposure=True, exposure=False, width=800, height=600, fps=10) -> None:
        ConfigurableCameraMixin.__init__(self)
        TransformableCameraMixin.__init__(self)
        Camera.__init__(self, id=id, name=name, connect_after_init=connect_after_init, streaming=streaming)
        self.device = None
        self.detect = False
        self.color = None

        self._register_parameter(name='auto_exposure', setter=self.set_exposure,
                                 getter=self.get_exposure, default_value=auto_exposure)
        self._register_parameter(name='exposure', setter=self.set_exposure,
                                 getter=self.get_exposure, default_value=exposure)
        self._register_parameter(name='width', setter=self.set_width, getter=self.get_width, default_value=width)
        self._register_parameter(name='height', setter=self.set_height, getter=self.get_height, default_value=height)
        self._register_parameter(name='fps', setter=self.set_fps, getter=self.get_fps, default_value=fps)

    def __to_dict__(self) -> dict[str, Any]:
        camera_dict = {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
            'parameters': {}
        }
        for param_name, param in self._parameters.items():
            camera_dict['parameters'][param_name] = param.value
        return camera_dict

    @staticmethod
    def from_dict(data: dict[str, Any]) -> 'UsbCamera':
        if 'parameters' not in data:
            data['parameters'] = {}
        fps = data['parameters'].get('fps', None)
        auto_exposure = data['parameters'].get('auto_exposure', None)
        exposure = data['parameters'].get('exposure', None)
        width = data['parameters'].get('width', None)
        height = data['parameters'].get('height', None)

        camera = UsbCamera(id=data['id'], name=data['name'], connect_after_init=data['connect_after_init'],
                           streaming=data['streaming'], auto_exposure=auto_exposure, exposure=exposure,
                           width=width, height=height, fps=fps)
        return camera

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if self.is_connected:
            return
        device = await UsbDevice.from_uid(self.id)
        if device is None:
            return

        self.device = device
        await self._apply_all_parameters()
        await self._update_parameter_values()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None
        await rosys.run.io_bound(self.device.capture.release)
        self.device = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None
        assert self.device.capture is not None

        _, captured_image = self.device.capture.read()
        if captured_image is None:
            return

        if 'MJPG' in self.device.video_formats:
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
        if not self.is_connected:
            return

        device = self.device

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if auto and not is_auto_exposure:
                # self.log.info(f'activating auto-exposure for {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                await self.set_exposure(self._parameters['exposure'].value)

    async def set_exposure(self, value: int) -> None:
        if not self.is_connected:
            return

        device = self.device
        assert device.capture is not None

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
                if value != exposure:
                    device.capture.set(cv2.CAP_PROP_EXPOSURE, int(value * device.exposure_max))

    async def get_auto_exposure(self) -> Optional[bool]:
        if not self.is_connected:
            return None
        device = self.device
        return device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    async def get_exposure(self) -> Optional[int]:
        if not self.is_connected:
            return None
        device = self.device
        if not device.has_manual_exposure:
            return None
        return device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max

    async def set_width(self, width: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    async def get_width(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    async def set_height(self, height: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    async def get_height(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    async def set_fps(self, fps: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FPS, fps)

    async def get_fps(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FPS))
