from typing import Any, Optional

from typing_extensions import Self

from ... import rosys
from ..camera import TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image
from ..image_rotation import ImageRotation
from ..rtsp_camera.arp_scan import find_ip
from .mjpeg_device import MjpegDevice


class MjpegCamera(TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 image_grab_interval: float = 0.1,
                 base_path_overwrite: str | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 **kwargs: Any,
                 ) -> None:
        super().__init__(id=id, name=name, connect_after_init=connect_after_init, streaming=streaming,
                         image_grab_interval=image_grab_interval, base_path_overwrite=base_path_overwrite, **kwargs)
        self.username = username
        self.password = password
        self.device: Optional[MjpegDevice] = None

    def to_dict(self) -> dict:
        return super().to_dict() | {
            'username': self.username,
            'password': self.password,
        }

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(**data)

    @property
    def is_connected(self) -> bool:
        return self.device is not None and self.device.capture_task is not None

    async def connect(self) -> None:
        if self.is_connected:
            return

        ip = await find_ip(self.id)
        if ip is None:
            return

        self.device = MjpegDevice(self.id, ip, username=self.username, password=self.password)

    async def disconnect(self) -> None:
        if self.device is None:
            return

        self.device.shutdown()
        self.device = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None

        image = self.device.capture()
        if image is None:
            return

        if self.crop or self.rotation != ImageRotation.NONE:
            image = await rosys.run.cpu_bound(process_jpeg_image, image, self.rotation, self.crop)
        if image is None:
            return
        try:
            final_image_resolution = get_image_size_from_bytes(image)
        except ValueError:
            return

        self._add_image(Image(camera_id=self.id, data=image, time=rosys.time(), size=final_image_resolution))
