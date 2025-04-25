import abc
import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Literal
from uuid import uuid4

from ..event import Event
from .detections import Category, Detections
from .image import Image


class Autoupload(Enum):
    """Configuration options for image auto-upload behavior to the Learning Loop"""

    FILTERED = 'filtered'
    """Upload mode for images with novel detections within uncertainty thresholds (default)"""

    DISABLED = 'disabled'
    """Upload mode where no images are auto-uploaded"""

    ALL = 'all'
    """Upload mode where every processed image is uploaded"""


class DetectorException(Exception):
    """An exception that is raised by a detector."""


@dataclass(slots=True, kw_only=True)
class DetectorInfo:
    """Information about the detector."""

    operation_mode: str
    """The operation mode of the detector node."""

    state: Literal['idle', 'online', 'detecting'] | None
    """The state of the detector node."""

    organization: str | None = None
    """The owner organization of the model."""

    project: str | None = None
    """The project of the model."""

    current_version: str | None = None
    """The currently used version of the model."""

    categories: list[Category] = field(default_factory=list)
    """The categories used in the model."""

    resolution: int | None = None
    """The resolution of the model (width and height of the image after preprocessing in pixels)."""

    target_version: str | None = None
    """The target version of the detector node."""

    version_control: Literal['follow_loop', 'specific_version', 'pause']
    """The version control mode of the detector node."""


@dataclass(slots=True, kw_only=True)
class ModelVersioningInfo:
    current_version: str
    """The version of the model currently used by the detector."""

    target_version: str
    """The target model version set in the detector."""

    loop_version: str
    """The target model version specified by the loop."""

    local_versions: list[str]
    """The locally available versions of the model."""

    version_control: str
    """The version control mode."""


class Detector(abc.ABC):
    """A detector allows detecting objects in images.

    It also holds an upload queue for sending images with uncertain results to an active learning infrastructure
    like the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).
    """

    def __init__(self, *, name: str | None = None) -> None:
        self.name = name or str(uuid4())

        self.NEW_DETECTIONS = Event[Image]()
        """detection on an image is completed (argument: image)"""

        self.log = logging.getLogger('rosys.detector')

    @abc.abstractmethod
    async def detect(self,
                     image: Image,
                     *,
                     autoupload: Autoupload = Autoupload.FILTERED,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> Detections | None:
        """Runs detections on the image and fills the ``image.detections`` property.

        The parameters ``tags``, ``source``, and ``creation_date`` are added as metadata if the image is uploaded.

        Note that the hardware detector uses a lazy strategy to schedule the inference tasks.
        In particular a queue with a maximum size of 1 is used.
        This means if the detector is busy, the image is not processed immediately, but queued up.
        If the ``detect`` function is called again, the queued image is dropped and the new image is queued instead.
        In this case this method returns ``None``.

        :return: the detections found in the image.
        :raises DetectorException: if the detection fails.
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

        :raises DetectorException: if the upload fails.
        """

    @abc.abstractmethod
    async def fetch_detector_info(self) -> DetectorInfo:
        """Retrieve information about the detector.

        :return: information about the detector.
        :raises DetectorException: if the about information cannot be retrieved.
        """

    @abc.abstractmethod
    async def fetch_model_version_info(self) -> ModelVersioningInfo:
        """Retrieve information about the model version and versioning mode.

        :return: the information about the model versioning as data class.
        :raises DetectorException: if the detector is not connected or the information cannot be retrieved.
        """

    @abc.abstractmethod
    async def set_model_version(self, version: Literal['follow_loop', 'pause'] | str) -> None:
        """Set the model version or versioning mode.

        Set to "follow_loop" to automatically update the model version to the latest version in the learning loop.
        Set to "pause" to stop automatic updates and keep the current model version.
        Set to a version number (e.g. "1.2") to use a specific version.

        :raises DetectorException: if the version control mode is not valid or the version could not be set.
        """
