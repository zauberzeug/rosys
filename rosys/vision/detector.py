
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
from .uploads import Uploads


class Autoupload(Enum):
    '''Configures the auto-submitting of images to the Learning Loop'''

    FILTERED = 'filtered'
    '''only submit images with novel detections and in an uncertainty range (this is the default)'''

    DISABLED = 'disabled'
    '''no auto-submitting'''

    ALL = 'all'
    '''submit all images which are run through the detector'''


class DetectorException(Exception):
    '''An exception that is raised by a detector.'''


@dataclass(slots=True, kw_only=True)
class DetectorInfo:
    '''Information about the detector.'''
    operation_mode: str = field(
        metadata={'description': 'The operation mode of the detector node'})
    state: str | None = field(
        metadata={'description': 'The state of the detector node', 'example': 'idle, online, detecting'})
    organization: str | None = field(
        default=None,
        metadata={'description': 'The owner organization of the model.'})
    project: str | None = field(
        default=None,
        metadata={'description': 'The project of the model.'})
    current_version: str | None = field(
        default=None,
        metadata={'description': 'The currently used version of the model.'})
    categories: list[Category] = field(
        default_factory=list,
        metadata={'description': 'The categories used in the model.'})
    resolution: int | None = field(
        default=None,
        metadata={'description': 'The resolution of the model (width and height of the image after preprocessing in pixels).'})
    target_version: str | None = field(
        default=None,
        metadata={'description': 'The target version of the detector node'})
    version_control: str = field(
        metadata={'description': 'The version control mode of the detector node',
                  'example': 'follow_loop, specific_version, pause'})


@dataclass(slots=True, kw_only=True)
class ModelVersioningInfo:
    current_version: str = field(metadata={'description': 'The version of the model currently used by the detector.'})
    target_version: str = field(metadata={'description': 'The target model version set in the detector.'})
    loop_version: str = field(metadata={'description': 'The target model version specified by the loop.'})
    local_versions: list[str] = field(metadata={'description': 'The locally available versions of the model.'})
    version_control: str = field(metadata={'description': 'The version control mode.'})


class Detector(abc.ABC):
    '''A detector allows detecting objects in images.

    It also holds an upload queue for sending images with uncertain results to an active learning infrastructure like the [Zauberzeug Learning Loop](https://zauberzeug.com/products/learning-loop).
    '''

    def __init__(self, *, name: str | None = None) -> None:
        self.name = name or str(uuid4())
        self.NEW_DETECTIONS = Event()
        '''detection on an image is completed (argument: image)'''
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
        '''Runs detections on the image. Afterwards the `image.detections` property is filled.

        The parameters `tags`, `source`, and `creation_date` are added as metadata if the image is uploaded.

        Returns:
            Detections: the detections found in the image.

        Raises:
            DetectorException: if the detection fails.
        '''

    @abc.abstractmethod
    async def upload(self,
                     image: Image,
                     *,
                     tags: list[str] | None = None,
                     source: str | None = None,
                     creation_date: datetime | str | None = None,
                     ) -> None:
        '''Uploads the image to the Learning Loop.

        The parameters `tags`, `source`, and `creation_date` are added as metadata.
        If the image has detections, they are also uploaded.

        Raises:
            DetectorException: if the upload fails.
        '''

    @abc.abstractmethod
    async def fetch_detector_info(self) -> DetectorInfo:
        '''Retrieve information about the detector.

        Returns:
            DetectorInfo: information about the detector.

        Raises:
            DetectorException: if the about information cannot be retrieved.
        '''

    @abc.abstractmethod
    async def fetch_model_version_info(self) -> ModelVersioningInfo:
        '''Retrieve information about the model version and versioning mode.

        Returns:
            ModelVersioningInfo: the information about the model versioning as data class.

        Raises:
            DetectorException: if the detector is not connected or the information cannot be retrieved.
        '''

    @abc.abstractmethod
    async def set_model_version(self, version: Literal['follow_loop', 'pause'] | str) -> None:
        '''Set the model version or versioning mode.

        Set to 'follow_loop' to automatically update the model version to the latest version in the learning loop.
        Set to 'pause' to stop automatic updates and keep the current model version.
        Set to a version number (e.g. '1.2') to use a specific version.

        Raises:
            DetectorException: if the version control mode is not valid or the version could not be set.
        '''
