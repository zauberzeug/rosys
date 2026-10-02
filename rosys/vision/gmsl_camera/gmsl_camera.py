from __future__ import annotations

import logging
import shutil
from typing import Any

from ... import rosys
from ..camera.calibratable_camera import CalibratableCamera
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image, ImageArray
from ..image_processing import process_ndarray_image
from ..image_rotation import ImageRotation
from .gmsl_device import GmslDevice


class GmslCamera(ConfigurableCamera, TransformableCamera, CalibratableCamera):
    """A GMSL2/FPD-Link camera connected through a deserializer board.

    The hardware is located by its Argus ``sensor_id`` (the GMSL port on the board), while ``id``
    is the stable identity used for persistence and image tagging, so a camera can move between
    ports without losing its persisted state. ``id`` defaults to ``gmsl-<sensor_id>``.
    """

    def __init__(self,
                 *,
                 sensor_id: int = 0,
                 id: str | None = None,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 auto_exposure: bool = True,
                 exposure: float = 0.01,
                 auto_gain: bool = True,
                 gain: float = 1.0,
                 fps: int = 30,
                 width: int = 1920,
                 height: int = 1200,
                 **kwargs) -> None:
        super().__init__(id=id or f'gmsl-{sensor_id}',
                         name=name,
                         connect_after_init=connect_after_init,
                         **kwargs)
        self.log = logging.getLogger(f'rosys.vision.gmsl_camera.{self.id}')
        self.sensor_id = sensor_id
        self.device: GmslDevice | None = None

        self._register_parameter('auto_exposure', self._get_auto_exposure, self._set_auto_exposure, auto_exposure)
        self._register_parameter('exposure', self._get_exposure, self._set_exposure, exposure)
        self._register_parameter('auto_gain', self._get_auto_gain, self._set_auto_gain, auto_gain)
        self._register_parameter('gain', self._get_gain, self._set_gain, gain)
        self._register_parameter('fps', self._get_fps, self._set_fps, fps)
        self._register_parameter('width', self._get_width, self._set_width, width)
        self._register_parameter('height', self._get_height, self._set_height, height)

    def to_dict(self) -> dict[str, Any]:
        return super().to_dict() | {
            'sensor_id': self.sensor_id,
        } | {
            name: param.value for name, param in self._parameters.items()
        }

    @property
    def is_connected(self) -> bool:
        return self.device is not None and self.device.is_connected

    @property
    def is_active(self) -> bool:
        return self.device is not None and self.device.is_active

    async def connect(self) -> None:
        async with self._device_connection():
            if self.device is not None:
                if self.device.is_active:
                    return
                await self._tear_down_device()
            if shutil.which('gst-launch-1.0') is None:
                self.log.warning('cannot connect camera %s: gst-launch-1.0 is not available '
                                 '(requires a Jetson with the Argus GStreamer stack)', self.id)
                return
            self.device = GmslDevice(
                self.sensor_id,
                on_new_image_data=self._handle_new_image_data,
                on_connect=self._apply_all_parameters,
                auto_exposure=self._parameters['auto_exposure'].value,
                exposure=self._parameters['exposure'].value,
                auto_gain=self._parameters['auto_gain'].value,
                gain=self._parameters['gain'].value,
                fps=self._parameters['fps'].value,
                width=self._parameters['width'].value,
                height=self._parameters['height'].value,
                reconnect_interval=self.reconnect_interval,
            )
            self.log.info('connecting camera %s (sensor-id %s)', self.id, self.sensor_id)

    async def disconnect(self) -> None:
        async with self._device_connection():
            await self._tear_down_device()

    async def _tear_down_device(self) -> None:
        """Tear down the device. The caller must hold `device_connection_lock`."""
        if self.device is None:
            return
        await self.device.shutdown()
        self.device = None
        self.log.info('camera %s: disconnected', self.id)

    async def _handle_new_image_data(self, image_array: ImageArray, timestamp: float) -> None:
        processed: ImageArray | None = image_array
        if self.crop or self.rotation != ImageRotation.NONE:
            processed = await rosys.run.cpu_bound(process_ndarray_image, image_array, self.rotation, self.crop)
        if processed is None:
            return
        image = Image.from_array(processed, camera_id=self.id, time=timestamp)
        self._add_image(image)

    def _set_device_value(self, name: str, value: Any) -> None:
        assert self.device is not None
        if getattr(self.device, name) == value:
            return  # reapplying the cached parameters after a reconnect must not restart the pipeline
        setattr(self.device, name, value)
        self.device.request_restart()

    def _set_auto_exposure(self, value: bool) -> None:
        self._set_device_value('auto_exposure', value)

    def _get_auto_exposure(self) -> bool:
        assert self.device is not None
        return self.device.auto_exposure

    def _set_exposure(self, value: float) -> None:
        self._set_device_value('exposure', value)

    def _get_exposure(self) -> float:
        assert self.device is not None
        return self.device.exposure

    def _set_auto_gain(self, value: bool) -> None:
        self._set_device_value('auto_gain', value)

    def _get_auto_gain(self) -> bool:
        assert self.device is not None
        return self.device.auto_gain

    def _set_gain(self, value: float) -> None:
        self._set_device_value('gain', value)

    def _get_gain(self) -> float:
        assert self.device is not None
        return self.device.gain

    def _set_fps(self, value: int) -> None:
        self._set_device_value('fps', value)

    def _get_fps(self) -> int:
        assert self.device is not None
        return self.device.fps

    def _set_width(self, value: int) -> None:
        self._set_device_value('width', value)

    def _get_width(self) -> int:
        assert self.device is not None
        return self.device.width

    def _set_height(self, value: int) -> None:
        self._set_device_value('height', value)

    def _get_height(self) -> int:
        assert self.device is not None
        return self.device.height
