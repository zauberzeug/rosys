from dataclasses import dataclass, field
from datetime import datetime

from rosys import persistence

from .image import Image


@dataclass(slots=True, kw_only=True)
class Uploads:
    minimal_minutes_between_uploads: float = 1.0
    last_upload: datetime = datetime.fromtimestamp(0)
    queue: dict[str, Image] = field(default_factory=dict, metadata=persistence.exclude)
    priority_queue: dict[str, Image] = field(default_factory=dict, metadata=persistence.exclude)

    def mark(self, image: Image, *, force: bool = False) -> None:
        '''Mark an image for upload. Set force=True to circumvent minimal_minutes_between_uploads.'''
        if force:
            self.priority_queue[image.id] = image
        else:
            self.queue[image.id] = image

    def get_queued(self) -> list[Image]:
        return [image for image in self.queue.values() if image.data]

    def get_priority_queued(self) -> list[Image]:
        return [image for image in self.priority_queue.values() if image.data]
