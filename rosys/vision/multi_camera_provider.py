import logging

from .camera import Camera
from .camera_provider import CameraProvider


class MultiCameraProvider(CameraProvider):
    """A multi-camera provider combines multiple camera providers into one.

    This is useful if another module requires a single camera provider but the robot has multiple camera sources like USB and WiFi cameras.
    """

    def __init__(self, *camera_providers: CameraProvider) -> None:
        super().__init__()
        self.providers = camera_providers
        self.log = logging.getLogger('rosys.multi_camera_provider')
        for camera_provider in self.providers:
            camera_provider.NEW_IMAGE.register(self.NEW_IMAGE.emit)
            camera_provider.CAMERA_ADDED.register(self.CAMERA_ADDED.emit)
            camera_provider.CAMERA_REMOVED.register(self.CAMERA_REMOVED.emit)

    async def update_device_list(self) -> None:
        for provider in self.providers:
            try:
                await provider.update_device_list()
            except Exception as e:
                self.log.error('Error while scanning for cameras in "%s": %s', provider.__class__.__name__, e)

    def request_backup(self) -> None:
        for provider in self.providers:
            provider.request_backup()

    @property
    def cameras(self) -> dict[str, Camera]:
        return {id: camera for provider in self.providers for id, camera in provider.cameras.items()}

    def remove_camera(self, camera_id: str) -> None:
        for provider in self.providers:
            if camera_id in provider.cameras:
                provider.remove_camera(camera_id)

    def add_camera(self, camera) -> None:
        raise NotImplementedError('Adding cameras to a MultiCameraProvider is not supported')
