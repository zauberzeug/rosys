from .camera import Camera


class UsbCamera(Camera):
    capture: bool = True
    detect: bool = False
