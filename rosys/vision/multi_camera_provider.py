from .camera import Camera
from .camera_provider import CameraProvider


class MultiCameraProvider(CameraProvider):
    '''A multi-camera provider combines multiple camera providers into one.

    This is useful if another module requires a single camera provider but the robot has multiple camera sources like USB and WiFi cameras.
    '''

    def __init__(self, *camera_providers: CameraProvider) -> None:
        self.providers = camera_providers

    @property
    def cameras(self) -> dict[str, Camera]:
        return {id: camera for provider in self.providers for id, camera in provider.cameras.items()}
