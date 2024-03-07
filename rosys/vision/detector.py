import abc
import logging
from enum import Enum
from typing import Optional
from uuid import uuid4

from ..event import Event
from .image import Image
from .uploads import Uploads


class Autoupload(Enum):
    """Configures the auto-submitting of images to the Learning Loop"""

    FILTERED = 'filtered'
    """only submit images with novel detections and in an uncertainty range (this is the default)"""

    DISABLED = 'disabled'
    """no auto-submitting"""

    ALL = 'all'
    """submit all images which are run through the detector"""


class Detector(abc.ABC):
    """A detector allows detecting objects in images.

    It also holds an upload queue for sending images with uncertain results to an active learning infrastructure like the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).
    """

    def __init__(self, *, name: Optional[str] = None) -> None:
        self.name = name or str(uuid4())
        self.NEW_DETECTIONS = Event()
        """detection on an image is completed (argument: image)"""
        self.log = logging.getLogger('rosys.detector')
        self.uploads = Uploads()

    @abc.abstractmethod
    async def detect(self, image: Image, autoupload: Autoupload = Autoupload.FILTERED, tags: list[str] = []) -> None:
        """Runs detections on the image. Afterwards the `image.detections` property is filled."""

    @abc.abstractmethod
    async def upload(self, image: Image, *, tags: list[str] = []) -> None:
        """Uploads the image to the Learning Loop. 

        The `tags` are added to the image.
        If the image has detections, they are also uploaded.
        """
