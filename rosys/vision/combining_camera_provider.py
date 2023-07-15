from .camera import Camera
from .camera_provider import CameraProvider


class CombiningCameraProvider(CameraProvider):

    def __init__(self, camera_providers: list[CameraProvider] = []):
        super().__init__()
        self.camera_providers = camera_providers

    @property
    def cameras(self) -> dict[str, Camera]:
        return {camera.id: camera for provider in self.camera_providers for camera in provider.cameras.values()}
