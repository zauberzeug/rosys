from .detections import Category
from dataclasses import field
import abc
import logging
from datetime import datetime
from enum import Enum
from uuid import uuid4

from ..event import Event
from .detections import Detections
from .image import Image
from .uploads import Uploads

from dataclasses import dataclass


class Autoupload(Enum):
    """Configures the auto-submitting of images to the Learning Loop"""

    FILTERED = 'filtered'
    """only submit images with novel detections and in an uncertainty range (this is the default)"""

    DISABLED = 'disabled'
    """no auto-submitting"""

    ALL = 'all'
    """submit all images which are run through the detector"""


class DetectorException(Exception):
    """An exception that is raised by a detector."""


@dataclass(slots=True, kw_only=True)
class DetectorInfo:
    """Information about the detector."""
    operation_mode: str = field(
        metadata={"description": "The operation mode of the detector node"})
    state: str | None = field(
        metadata={"description": "The state of the detector node", "example": "idle, online, detecting"})
    organization: str | None = field(
        default=None,
        metadata={"description": "The owner organization of the model."})
    project: str | None = field(
        default=None,
        metadata={"description": "The project of the model."})
    current_version: str | None = field(
        default=None,
        metadata={"description": "The currently used version of the model."})
    categories: list[Category] = field(
        default_factory=list,
        metadata={"description": "The categories used in the model."})
    resolution: int | None = field(
        default=None,
        metadata={"description": "The resolution of the model (width and height of the image after preprocessing in pixels)."})
    target_version: str | None = field(
        default=None,
        metadata={"description": "The target version of the detector node"})
    version_control: str = field(
        metadata={"description": "The version control mode of the detector node",
                  "example": "follow_loop, specific_version, pause"})


class Detector(abc.ABC):
    """A detector allows detecting objects in images.

    It also holds an upload queue for sending images with uncertain results to an active learning infrastructure like the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).
    """

    def __init__(self, *, name: str | None = None) -> None:
        self.name = name or str(uuid4())
        self.NEW_DETECTIONS = Event()
        """detection on an image is completed (argument: image)"""
        self.log = logging.getLogger('rosys.detector')
        self.uploads = Uploads()

    @abc.abstractmethod
    async def detect(self,
                     image: Image,
                     *,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> Detections | None:
        """Runs detections on the image. Afterwards the `image.detections` property is filled.

        The parameters `tags`, `source`, and `creation_date` are added as metadata if the image is uploaded.

        Returns:
            Detections: the detections found in the image.

        Raises:
            DetectorException: if the detection fails.
        """

    @abc.abstractmethod
    async def upload(self,
                     image: Image,
                     *,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> None:
        """Uploads the image to the Learning Loop.

        The parameters `tags`, `source`, and `creation_date` are added as metadata.
        If the image has detections, they are also uploaded.

        Raises:
            DetectorException: if the upload fails.
        """

    @abc.abstractmethod
    async def fetch_detector_information(self) -> DetectorInfo:
        """Get information about the detector.

        Returns:
            DetectorInfo: information about the detector.

        Raises:
            DetectorException: if the about information cannot be retrieved.
        """
