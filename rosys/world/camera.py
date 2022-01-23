
from typing import List
from pydantic import BaseModel, Field
from .image import Image

no_img_placeholder = Image.create_placeholder('no image')


class Camera(BaseModel):
    id: str
    capture: bool = True
    detect: bool = False
    images: List[Image] = Field([no_img_placeholder], exclude=True)

    @property
    def latest_image_uri(self):
        return f'camera/{self.id}/{self.images[-1].time}'
