from typing import Literal
from .camera import Camera


class UsbCamera(Camera):
    type: Literal['usb_camera'] = 'usb_camera'
    capture: bool = True
    detect: bool = False
