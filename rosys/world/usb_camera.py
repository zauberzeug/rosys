from typing import Optional

from pydantic import Field

from .camera import Camera
from .image import ImageSize


class UsbCamera(Camera):
    capture: bool = True
    detect: bool = False
    connected: bool = False
    resolution: Optional[ImageSize] = None
    exposure: Optional[float] = None
    color: Optional[str] = None
