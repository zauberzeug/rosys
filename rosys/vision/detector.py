import abc
from typing import Optional

from aenum import Enum, auto

from ..event import Event
from .detections import Detections
from .image import Image
from .uploads import Uploads


class Autoupload(Enum, init='value __doc__'):
    '''Configures the auto-submitting of images to the Learning Loop'''

    def _generate_next_value_(name: str, start, count, last_values) -> str:
        '''uses enum name as value when calling auto()'''
        return name.lower()

    FILTERED = auto(), 'only submit images with novel detections and in an uncertainty range (this is the default)'
    DISABLED = auto(), 'no auto-submitting'
    ALL = auto(), 'submit all images which are run through the detector'


class Detector(abc.ABC):
    '''A detector allows detecting objects in images.

    It also holds an upload queue for sending images with uncertain results to an active learning infrastructure like the [Zauberzeug Learning Loop](https://zauberzeug.com/learning-loop.html).
    '''

    def __init__(self) -> None:
        self.NEW_DETECTIONS = Event()
        '''detection on an image is completed (argument: image)'''

    @abc.abstractmethod
    async def detect(self, image: Image, autoupload: Autoupload = Autoupload.FILTERED) -> Optional[Detections]:
        return

    @property
    def uploads(self) -> Uploads:
        return
