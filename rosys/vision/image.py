from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Optional

import PIL.Image
import PIL.ImageDraw

from .detections import Detections


@dataclass(slots=True, kw_only=True)
class ImageSize:
    width: int
    height: int

    @property
    def tuple(self) -> tuple[float, float]:
        return (self.width, self.height)


@dataclass(slots=True, kw_only=True)
class Image:
    camera_id: str
    size: ImageSize
    time: float  # time of recording
    data: Optional[bytes] = None
    detections: Optional[Detections] = None
    is_broken: Optional[bool] = None

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @staticmethod
    def create_placeholder(text: str, time: Optional[float] = None, camera_id: Optional[str] = None) -> Image:
        img = PIL.Image.new('RGB', (260, 200), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        image = Image(
            camera_id=camera_id or 'no_cam_id',
            time=time or 0,
            size=ImageSize(width=img.width, height=img.height),
        )
        image.data = bytesio.getvalue()
        return image
