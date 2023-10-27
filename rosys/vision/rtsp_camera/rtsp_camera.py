import io
import logging
import os
import sys
from typing import Any, Optional, Self

import imgsize
import netifaces
import PIL

from ... import rosys
from ...geometry import Rectangle
from ..camera.configurable_camera import ConfigurableCameraMixin
from ..camera.transformable_camera import TransformableCameraMixin
from ..image import Image, ImageSize
from ..image_processing import PeekableBytesIO, process_jpeg_image
from ..image_rotation import ImageRotation
from .rtsp_device import RtspDevice
from .vendors import VendorType, mac_to_vendor


def process_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


async def find_ip_from_mac(mac: str) -> Optional[str]:
    """Find the IP address of a device with a given MAC address."""
    for interface in netifaces.interfaces():
        if sys.platform.startswith('darwin'):
            arpscan_cmd = 'arp-scan'
        else:
            arpscan_cmd = '/usr/sbin/arp-scan'  # TODO is this necessary? sbin should be in path
        if os.getuid() != 0:
            arpscan_cmd = f'sudo {arpscan_cmd}'
        cmd = f'{arpscan_cmd} -I {interface} --localnet'
        output = await rosys.run.sh(cmd, timeout=10)
        if output is None:
            continue
        for line in output.splitlines():
            infos = line.split()
            if len(infos) < 2:
                continue
            if infos[1] == mac:
                return infos[0]
    return None


class RtspCamera(ConfigurableCameraMixin, TransformableCameraMixin):
    device: Optional[RtspDevice]
    jovision_profile: int

    detect: bool
    authorized: bool

    def __init__(self,
                 *,
                 id: str,
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

        self.device = None
        self.jovision_profile = jovision_profile
        self.authorized = True

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
            'parameters': {name: param.value for name, param in self._parameters.items()},
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            id=data['id'],
            name=data['name'],
            connect_after_init=data['connect_after_init'],
            streaming=data['streaming'],
            goal_fps=data.get('parameters', {}).get('fps'),
            jovision_profile=data.get('parameters', {}).get('jovision_profile'),
        )

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
        ip = await find_ip_from_mac(self.id)
        if ip is None:
            raise Exception(f'could not find IP address for {self.id}')

        device = RtspDevice(mac=self.id, ip=ip, jovision_profile=self.jovision_profile)
        if device is None:
            logging.warning(f'could not create device for {self.id}')
            return

        self.device = device
        # await self._apply_all_parameters()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return
        assert self.device is not None
        self.device.shutdown()
        self.device = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None
        assert self.device is not None

        image = self.device.capture()
        if not image:
            return
        final_image_bytes = await rosys.run.cpu_bound(process_jpeg_image, image, self.rotation, self.crop)

        try:
            with PeekableBytesIO(final_image_bytes) as f:
                width, height = imgsize.get_size(f)
        except imgsize.UnknownSize:
            return

        final_image_resolution = ImageSize(width=width, height=height)

        self._add_image(Image(time=rosys.time(), camera_id=self.id,
                        size=final_image_resolution, data=final_image_bytes))

    async def set_fps(self, fps: int) -> None:
        if self.device is None or self.device.settings_interface is None:
            return
        self.device.settings_interface.set_fps(stream_id=self.jovision_profile, fps=fps)
        await self.reconnect()

    async def get_fps(self) -> Optional[int]:
        if self.device is None or self.device.settings_interface is None:
            return None
        fps = self.device.settings_interface.get_fps(stream_id=self.jovision_profile)
        return fps

    async def set_jovision_profile(self, profile: int) -> None:
        if self.device is None:
            return
        self.jovision_profile = profile
        await self.reconnect()

    async def get_jovision_profile(self) -> Optional[int]:
        if self.device is None:
            return None
        return self.jovision_profile

    @staticmethod
    def known_vendor(mac: str) -> bool:
        return mac_to_vendor(mac) != VendorType.OTHER

    async def _apply_parameters(self, new_values: dict[str, Any]) -> None:
        await super()._apply_parameters(new_values)
        await self.reconnect()
