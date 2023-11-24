import logging
from typing import Any, Optional, Self

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image
from .arp_scan import find_ip
from .rtsp_device import RtspDevice


class RtspCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: Optional[str] = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 goal_fps: int = 5,
                 jovision_profile: int = 1,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         streaming=streaming,
                         **kwargs)

        self.device: Optional[RtspDevice] = None
        self.jovision_profile: int = jovision_profile

        self._register_parameter(name='fps', getter=self.get_fps, setter=self.set_fps,
                                 min_value=1, max_value=30, step=1, default_value=goal_fps)
        self._register_parameter(name='jovision_profile', getter=self.get_jovision_profile, setter=self.set_jovision_profile,
                                 min_value=1, max_value=2, step=1, default_value=jovision_profile)

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
        return self.device is not None and self.device.capture_task is not None

    @property
    def url(self) -> Optional[str]:
        if not self.is_connected:
            return None
        assert self.device is not None

        return self.device.url

    async def connect(self) -> None:
        if self.is_connected:
            return

        ip = await find_ip(self.id)
        if ip is None:
            raise RuntimeError(f'could not find IP address for {self.id}')

        self.device = RtspDevice(mac=self.id, ip=ip, jovision_profile=self.jovision_profile)

        self._apply_all_parameters()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return
        logging.info(f'camera {self.id}: disconnect initialized...')

        assert self.device is not None
        self.device.shutdown()
        self.device = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return
        assert self.device is not None

        image_bytes = self.device.capture()

        if not image_bytes:
            return
        transformed_image_bytes = await rosys.run.cpu_bound(process_jpeg_image, image_bytes, self.rotation, self.crop)

        try:
            final_image_resolution = get_image_size_from_bytes(transformed_image_bytes)
        except ValueError:
            return

        image = Image(time=rosys.time(), camera_id=self.id, size=final_image_resolution, data=transformed_image_bytes)
        self._add_image(image)

    def set_fps(self, fps: int) -> None:
        if self.device is None or self.device.settings_interface is None:
            return
        self.device.settings_interface.set_fps(stream_id=self.jovision_profile, fps=fps)

    def get_fps(self) -> Optional[int]:
        if self.device is None or self.device.settings_interface is None:
            return None
        fps = self.device.settings_interface.get_fps(stream_id=self.jovision_profile)
        return fps

    def set_jovision_profile(self, profile: int) -> None:
        if self.device is None:
            return
        self.jovision_profile = profile

    def get_jovision_profile(self) -> Optional[int]:
        if self.device is None:
            return None
        return self.jovision_profile

    def _apply_parameters(self, new_values: dict[str, Any]) -> None:
        super()._apply_parameters(new_values)
        if not self.is_connected:
            assert self.device is not None
            self.device.restart_gstreamer()
