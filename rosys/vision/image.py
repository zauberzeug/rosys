from __future__ import annotations

import io
from dataclasses import dataclass
from typing import ClassVar, Optional

import PIL.Image
import PIL.ImageDraw

from .detections import Detections


@dataclass(slots=True, kw_only=True)
class ImageSize:
    width: int
    height: int

    @property
    def tuple(self) -> tuple[int, int]:
        return (self.width, self.height)


@dataclass(slots=True, kw_only=True)
class Image:
    camera_id: str
    size: ImageSize
    time: float  # time of recording
    data: Optional[bytes] = None
    detections: Optional[Detections] = None
    is_broken: Optional[bool] = None

    DEFAULT_PLACEHOLDER_SIZE: ClassVar[tuple[int, int]] = (260, 200)

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @classmethod
    def create_placeholder(cls, text: str, time: Optional[float] = None, camera_id: Optional[str] = None, shrink: int = 1) -> Image:
        h, w = cls.DEFAULT_PLACEHOLDER_SIZE
        img = PIL.Image.new('RGB', (h // shrink, w // shrink), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width / 2 - len(text) * 3, img.height / 2 - 5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        image = Image(
            camera_id=camera_id or 'no_cam_id',
            time=time or 0,
            size=ImageSize(width=img.width, height=img.height),
        )
        image.data = bytesio.getvalue()
        return image
