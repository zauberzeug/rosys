from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

from rosys import persistence

from .image import Image


@dataclass(slots=True, kw_only=True)
class UploadItem:
    image: Image
    detector_name: str

    def __hash__(self) -> int:
        return hash(self.image)


@dataclass(slots=True, kw_only=True)
class Uploads:
    minimal_minutes_between_uploads: float = 1.0
    last_upload: datetime = datetime.fromtimestamp(0)
    queue: set[UploadItem] = field(default_factory=set, metadata=persistence.exclude)
    priority_queue: set[UploadItem] = field(default_factory=set, metadata=persistence.exclude)

    def mark(self, image: Image, *, force: bool = False, detector_name: Optional[str] = None) -> None:
        '''Mark an image for upload. Set force=True to circumvent minimal_minutes_between_uploads.'''
        data = UploadItem(image=image, detector_name='detector' if detector_name is None else detector_name)
        if force:
            self.priority_queue.add(data)
        else:
            self.queue.add(data)

    def get_queued(self, detector_name: str) -> list[Image]:
        return [data.image for data in self.queue if data.detector_name == detector_name and data.image.data]

    def get_priority_queued(self, detector_name: str) -> list[Image]:
        return [data.image for data in self.priority_queue if data.detector_name == detector_name and data.image.data]
