from typing import Literal, Optional
from pydantic import Field
from .image import ImageSize
from .camera import Camera


class UsbCamera(Camera):
    type: Literal['usb_camera'] = 'usb_camera'
    capture: bool = True
    detect: bool = False
    resolution: Optional[ImageSize] = Field(None, exclude=True)
