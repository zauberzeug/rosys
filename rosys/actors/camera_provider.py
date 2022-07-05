import abc
from typing import Optional

from rosys import persistence

from ..event import Event
from ..runtime import runtime
from ..world import Camera


class CameraProvider(abc.ABC):

    CAMERA_ADDED = Event()
    '''a new camera has been discoverd (argument: new camera)'''

    def __init__(self) -> None:
        self.needs_backup: bool = False
        persistence.register(self)

    @property
    @abc.abstractmethod
    def cameras(self) -> dict[str, Camera]:
        return {}

    def add_camera(self, camera) -> None:
        self.cameras[camera.id] = camera
        self.CAMERA_ADDED.emit(camera)
        self.needs_backup = True

    def prune_images(self, max_age_seconds: Optional[float] = None):
        for camera in self.cameras.values():
            if max_age_seconds is None:
                camera.images.clear()
            else:
                while camera.images and camera.images[0].time < runtime.time - max_age_seconds:
                    del camera.images[0]
