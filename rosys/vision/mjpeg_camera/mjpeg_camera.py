from typing import Any, Coroutine, Optional

from ... import rosys
from ..camera import Camera, TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image
from ..image_rotation import ImageRotation
from .mjpeg_device import MjpegDevice


class MJpegCamera(TransformableCamera):
    def __init__(self, *, id: str, name: str | None = None, connect_after_init: bool = True, streaming: bool = True, image_grab_interval: float = 0.1, base_path_overwrite: str | None = None, **kwargs) -> None:
        super().__init__(id=id, name=name, connect_after_init=connect_after_init, streaming=streaming,
                         image_grab_interval=image_grab_interval, base_path_overwrite=base_path_overwrite, **kwargs)
        self.device: Optional[MjpegDevice] = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None

        image = self.device.capture()
        if image is None:
            return

        if self.crop or self.rotation != ImageRotation.NONE:
            image = await rosys.run.cpu_bound(process_jpeg_image, image, self.rotation, self.crop)
        try:
            final_image_resolution = get_image_size_from_bytes(image)
        except ValueError:
            return

        self._add_image(Image(camera_id=self.id, data=image, time=rosys.time(), size=final_image_resolution))
