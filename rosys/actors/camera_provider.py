import abc

from rosys import persistence

from ..event import Event
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
