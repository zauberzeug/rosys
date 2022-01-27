from typing import Optional
from pydantic import BaseModel
import PIL.Image
import PIL.ImageDraw
import io
from .detection import Detection
import urllib.parse


class ImageSize(BaseModel):
    width: int
    height: int


class Image(BaseModel):
    camera_id: str
    size: ImageSize
    time: float  # World time of recording
    data: Optional[bytes] = None
    detections: Optional[list[Detection]] = None

    @property
    def url(self) -> str:
        return f'camera/{urllib.parse.quote_plus(self.camera_id)}/{self.time}'

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @staticmethod
    def create_placeholder(text: str, time: float = None):
        img = PIL.Image.new('RGB', (260, 200), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        result = Image(camera_id='no_cam_id', time=time or 0, size=ImageSize(width=img.width, height=img.height))
        result.data = bytesio.getvalue()
        return result

    def __hash__(self):
        return hash(self.id)
