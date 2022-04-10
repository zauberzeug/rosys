from datetime import datetime

from pydantic import BaseModel, Field

from .image import Image


class UploadData(BaseModel):
    image: Image
    detector_name: str

    def __hash__(self) -> int:
        return hash(self.image)


class Upload(BaseModel):
    minimal_minutes_between_uploads: float = 1.0
    last_upload: datetime = datetime.fromtimestamp(0)
    queue: set[UploadData] = Field(set(), exclude=True)
    priority_queue: set[UploadData] = Field(set(), exclude=True)

    def mark(self, image: Image, *, force: bool = False, detector_name=None):
        '''Mark an image for upload. Set force=True to circumvent minimal_minutes_between_uploads.'''
        data = UploadData(image=image, detector_name=detector_name or 'detector')
        if force:
            self.priority_queue.add(data)
        else:
            self.queue.add(data)

    def get_queued(self, detector_name: str):
        return [data.image for data in self.queue if data.detector_name == detector_name and data.image.data]

    def get_priority_queued(self, detector_name: str):
        return [data.image for data in self.priority_queue if data.detector_name == detector_name and data.image.data]
