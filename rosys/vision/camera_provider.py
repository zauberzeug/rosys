import abc
import urllib.parse
from typing import Optional
from uuid import uuid4

import rosys

from ..event import Event
from .camera import Camera
from .image import Image
from .image_route import create_image_route


class CameraProvider(abc.ABC):
    '''A camera provider holds a dictionary of cameras and manages additions and removals.

    The camera dictionary should not be modified directly but by using the camera provider's methods.
    This way respective events are emitted and consistency can be taken care of.

    The camera provider also creates an HTTP route to access camera images.
    '''

    def __init__(self) -> None:
        self.CAMERA_ADDED = Event()
        '''a new camera has been added (argument: camera)'''

        self.CAMERA_REMOVED = Event()
        '''a camera has been removed (argument: camera id)'''

        self.NEW_IMAGE = Event()
        '''an new image is available (argument: image)'''

        self.base_path = f'images/{str(uuid4())}'
        create_image_route(self)

    @property
    @abc.abstractmethod
    def cameras(self) -> dict[str, Camera]:
        return {}

    @property
    def images(self) -> list[Image]:
        return sorted((i for c in self.cameras.values() for i in c.images), key=lambda i: i.time)

    def get_image_url(self, image: Image) -> str:
        return f'{self.base_path}/{urllib.parse.quote_plus(image.camera_id)}/{image.time}'

    def get_latest_image_url(self, camera: Camera) -> str:
        image = camera.latest_captured_image
        if image is None:
            return f'{self.base_path}/placeholder'
        return self.get_image_url(image)

    def add_camera(self, camera: Camera) -> None:
        self.cameras[camera.id] = camera
        self.CAMERA_ADDED.emit(camera)
        self.needs_backup = True

    def remove_camera(self, camera_id: str) -> None:
        del self.cameras[camera_id]
        self.CAMERA_REMOVED.emit(camera_id)
        self.needs_backup = True

    def remove_all_cameras(self) -> None:
        [self.remove_camera(camera_id) for camera_id in list(self.cameras)]

    def remove_calibration(self, camera_id: str) -> None:
        self.cameras[camera_id].calibration = None
        self.needs_backup = True

    def remove_all_calibrations(self) -> None:
        [self.remove_calibration(camera_id) for camera_id in self.cameras]

    def add_image(self, camera: Camera, image: Image) -> None:
        camera.images.append(image)
        self.NEW_IMAGE.emit(image)

    def prune_images(self, max_age_seconds: Optional[float] = None):
        for camera in self.cameras.values():
            if max_age_seconds is None:
                camera.images.clear()
            else:
                while camera.images and camera.images[0].time < rosys.time() - max_age_seconds:
                    del camera.images[0]
