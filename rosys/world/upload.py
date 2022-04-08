from datetime import datetime

from pydantic import BaseModel, Field

from .image import Image


class Upload(BaseModel):
    minimal_minutes_between_uploads: float = 1.0
    last_upload: datetime = datetime.fromtimestamp(0)
    queue: set[Image] = Field(set(), exclude=True)
    priority_queue: set[Image] = Field(set(), exclude=True)

    def mark(self, image: Image, *, force: bool = False):
        '''Mark an image for upload. Set force=True to circumvent minimal_minutes_between_uploads.'''
        if force:
            self.priority_queue.add(image)
        else:
            self.queue.add(image)
