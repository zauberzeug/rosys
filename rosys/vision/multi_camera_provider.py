from .camera import Camera
from .camera_provider import CameraProvider


class MultiCameraProvider(CameraProvider):

    def __init__(self, *camera_providers: CameraProvider) -> None:
        self.providers = camera_providers

    @property
    def cameras(self) -> dict[str, Camera]:
        return {id: camera for provider in self.providers for id, camera in provider.cameras.items()}
