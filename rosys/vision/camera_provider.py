import abc
from typing import Generic, Optional, TypeVar

from .. import rosys
from ..event import Event
from .camera.camera import Camera
from .image import Image

T = TypeVar('T', bound=Camera)


class CameraProvider(Generic[T], rosys.persistence.PersistentModule, metaclass=abc.ABCMeta):
    """A camera provider holds a dictionary of cameras and manages additions and removals.

    The camera dictionary should not be modified directly but by using the camera provider's methods.
    This way respective events are emitted and consistency can be taken care of.

    The camera provider also creates an HTTP route to access camera images.
    """

    def __init__(self) -> None:
        super().__init__()

        self.CAMERA_ADDED = Event()
        """a new camera has been added (argument: camera)"""

        self.CAMERA_REMOVED = Event()
        """a camera has been removed (argument: camera id)"""

        self.NEW_IMAGE = Event()
        """a new image is available (argument: image)"""

        self._cameras: dict[str, T] = {}

    def backup(self) -> dict:
        return {
            'cameras': {id: camera.to_dict() for id, camera in self._cameras.items()},
        }

    def restore(self, data: dict[str, dict]) -> None:
        rosys.persistence.replace_dict(self._cameras, Camera, data.get('cameras', {}))
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @property
    def cameras(self) -> dict[str, T]:
        return self._cameras

    @property
    def images(self) -> list[Image]:
        return sorted((i for c in self.cameras.values() for i in c.images), key=lambda i: i.time)

    def add_camera(self, camera: T) -> None:
        self._cameras[camera.id] = camera
        self.CAMERA_ADDED.emit(camera)
        camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)
        self.request_backup()

    def remove_camera(self, camera_id: str) -> None:
        del self.cameras[camera_id]
        self.CAMERA_REMOVED.emit(camera_id)
        self.request_backup()

    def remove_all_cameras(self) -> None:
        for camera_id in list(self.cameras):
            self.remove_camera(camera_id)

    def prune_images(self, max_age_seconds: Optional[float] = None):
        for camera in self.cameras.values():
            if max_age_seconds is None:
                camera.images.clear()
            else:
                while camera.images and camera.images[0].time < rosys.time() - max_age_seconds:
                    del camera.images[0]
