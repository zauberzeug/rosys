from pydantic import BaseModel

from .. import event
from .camera import Camera
from .upload import Upload
from .usb_camera import UsbCamera


class World(BaseModel):
    usb_cameras: dict[str, UsbCamera] = {}
    upload: Upload = Upload()
    needs_backup: bool = False

    @property
    def cameras(self) -> dict[str, Camera]:
        return self.usb_cameras

    async def add_usb_camera(self, camera: UsbCamera) -> None:
        self.usb_cameras[camera.id] = camera
        await event.call(event.Id.NEW_CAMERA, camera)
