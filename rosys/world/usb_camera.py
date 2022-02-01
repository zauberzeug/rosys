from typing import Optional
from pydantic import Field
from .image import ImageSize
from .camera import Camera


class UsbCamera(Camera):
    capture: bool = True
    detect: bool = False
    resolution: Optional[ImageSize] = Field(None, exclude=True)
