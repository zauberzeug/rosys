from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar, Optional, Self

import cv2
import numpy as np
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
    _detections: dict[str, Detections] = field(default_factory=dict)
    is_broken: Optional[bool] = None

    DEFAULT_PLACEHOLDER_SIZE: ClassVar[tuple[int, int]] = (320, 240)

    @property
    def detections(self) -> Optional[Detections]:
        if not self._detections:
            return None
        if len(self._detections) > 1:
            raise RuntimeError(
                f'Image has multiple detection types ({", ".join(self._detections.keys())}). Use `get_detections(type)` instead.')
        return list(self._detections.values())[0]

    def get_detections(self, detector_id: str) -> Optional[Detections]:
        return self._detections.get(detector_id)

    def set_detections(self, detector_id: str, detections: Detections) -> None:
        self._detections[detector_id] = detections

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @classmethod
    def create_placeholder(cls, text: str, time: Optional[float] = None, camera_id: Optional[str] = None, shrink: int = 1) -> Self:
        h, w = cls.DEFAULT_PLACEHOLDER_SIZE
        img = PIL.Image.new('RGB', (h // shrink, w // shrink), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width / 2 - len(text) * 3, img.height / 2 - 5), text, fill=(255, 255, 255))
        _, encoded_image = cv2.imencode('.png', np.array(img)[:, :, ::-1])  # NOTE: cv2 expects BGR
        return cls(
            camera_id=camera_id or 'no_cam_id',
            time=time or 0,
            size=ImageSize(width=img.width, height=img.height),
            data=encoded_image.tobytes(),
        )
