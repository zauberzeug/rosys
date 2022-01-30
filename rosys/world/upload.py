from pydantic import BaseModel, Field
from datetime import datetime
from .image import Image


class Upload(BaseModel):
    minimal_minutes_between_uploads = 1
    last_upload: datetime = datetime.fromtimestamp(0)
    queue: set[Image] = Field(set(), exclude=True)

    def mark(self, image: Image):
        '''Mark an image for upload. May still not get uploaded because minimal_minutes_between_uploads has not elapsed.'''
        self.queue.add(image)
