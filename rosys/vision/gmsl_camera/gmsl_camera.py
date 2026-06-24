from __future__ import annotations

import logging
import shutil
from typing import Any

from ... import rosys
from ..camera.calibratable_camera import CalibratableCamera
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image
from ..image_processing import process_ndarray_image
from ..image_rotation import ImageRotation
from .gmsl_device import GmslDevice


class GmslCamera(ConfigurableCamera, TransformableCamera, CalibratableCamera):
    """A GMSL2/FPD-Link camera connected through a deserializer board.

    Frames are captured through NVIDIA's Argus stack via `GmslDevice`, so the hardware ISP
    handles debayering, white balance and tone mapping. Exposure and gain can run on the ISP's
    auto algorithms or be pinned to fixed values.

    The hardware is located by its Argus ``sensor_id`` (the GMSL port on the board), while ``id``
    is the stable application-level identity used for persistence and image tagging.
    Keeping them separate lets a camera move between ports without losing its persisted state;
    ``id`` defaults to ``f'gmsl-{sensor_id}'`` when not given.
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

    async def connect(self) -> None:
        if self.is_connected:
            return
        if shutil.which('gst-launch-1.0') is None:
            self.log.warning('cannot connect camera %s: gst-launch-1.0 is not available '
                             '(requires a Jetson with the Argus GStreamer stack)', self.id)
            return
        self.device = GmslDevice(
            self.sensor_id,
            on_new_image_data=self._handle_new_image_data,
            auto_exposure=self._parameters['auto_exposure'].value,
            exposure=self._parameters['exposure'].value,
            auto_gain=self._parameters['auto_gain'].value,
            gain=self._parameters['gain'].value,
            fps=self._parameters['fps'].value,
            width=self._parameters['width'].value,
            height=self._parameters['height'].value,
        )
        self.log.info('connecting camera %s (sensor-id %s)', self.id, self.sensor_id)

    async def disconnect(self) -> None:
        if self.device is None:
            return
        await self.device.shutdown()
        self.device = None
        self.log.info('camera %s: disconnected', self.id)

    async def _handle_new_image_data(self, image_array, timestamp: float) -> None:
        if not self.is_connected:
            return
        if self.crop or self.rotation != ImageRotation.NONE:
            image_array = await rosys.run.cpu_bound(process_ndarray_image, image_array, self.rotation, self.crop)
        if image_array is None:
            return
        image = Image.from_array(image_array, camera_id=self.id, time=timestamp)
        self._add_image(image)

    def _set_auto_exposure(self, value: bool) -> None:
        assert self.device is not None
        self.device.auto_exposure = value
        self.device.request_restart()

    def _get_auto_exposure(self) -> bool:
        assert self.device is not None
        return self.device.auto_exposure

    def _set_exposure(self, value: float) -> None:
        assert self.device is not None
        self.device.exposure = value
        self.device.request_restart()

    def _get_exposure(self) -> float:
        assert self.device is not None
        return self.device.exposure

    def _set_auto_gain(self, value: bool) -> None:
        assert self.device is not None
        self.device.auto_gain = value
        self.device.request_restart()

    def _get_auto_gain(self) -> bool:
        assert self.device is not None
        return self.device.auto_gain

    def _set_gain(self, value: float) -> None:
        assert self.device is not None
        self.device.gain = value
        self.device.request_restart()

    def _get_gain(self) -> float:
        assert self.device is not None
        return self.device.gain

    def _set_fps(self, value: int) -> None:
        assert self.device is not None
        self.device.fps = value
        self.device.request_restart()

    def _get_fps(self) -> int:
        assert self.device is not None
        return self.device.fps

    def _set_width(self, value: int) -> None:
        assert self.device is not None
        self.device.width = value
        self.device.request_restart()

    def _get_width(self) -> int:
        assert self.device is not None
        return self.device.width

    def _set_height(self, value: int) -> None:
        assert self.device is not None
        self.device.height = value
        self.device.request_restart()

    def _get_height(self) -> int:
        assert self.device is not None
        return self.device.height
