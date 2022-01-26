from __future__ import annotations
from typing import Optional
from dataclasses import dataclass, field
import PIL.Image
import PIL.ImageDraw
import io
from .detection import Detection


@dataclass
class ImageSize:
    width: int
    height: int

    @property
    def tuple(self):
        return (self.width, self.height)


@dataclass
class Image:
    camera_id: str
    size: ImageSize
    time: float  # World time of recording
    data: Optional[bytes] = field(default=None, init=False)
    detections: Optional[list[Detection]] = field(default=None, init=False)

    @staticmethod
    def create_placeholder(text: str, time: float = None) -> Image:
        img = PIL.Image.new('RGB', (260, 200), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        image = Image(camera_id='no_cam_id', time=time or 0, size=ImageSize(width=img.width, height=img.height))
        image.data = bytesio.getvalue()
        return image
