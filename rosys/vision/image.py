from __future__ import annotations

import io
import warnings
from dataclasses import dataclass, field
from typing import ClassVar

import cv2
import numpy as np
import PIL.Image
import PIL.ImageDraw
from typing_extensions import Self

from .. import rosys
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
    data: bytes | None = None
    _detections: dict[str, Detections] = field(default_factory=dict)
    is_broken: bool | None = None
    tags: set[str] = field(default_factory=set)

    DEFAULT_PLACEHOLDER_SIZE: ClassVar[tuple[int, int]] = (320, 240)

    def __post_init__(self) -> None:
        if self.tags:
            warnings.warn('The "tags" field is deprecated and will be removed in a future version.',
                          DeprecationWarning, stacklevel=2)

    @property
    def detections(self) -> Detections | None:
        if not self._detections:
            return None
        if len(self._detections) > 1:
            raise RuntimeError(
                f'Image has multiple detection types ({", ".join(self._detections.keys())}). Use `get_detections(type)` instead.')
        return next(iter(self._detections.values()))

    def get_detections(self, detector_id: str) -> Detections | None:
        return self._detections.get(detector_id)

    def set_detections(self, detector_id: str, detections: Detections) -> None:
        self._detections[detector_id] = detections

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @classmethod
    def create_placeholder(cls, text: str, time: float | None = None, camera_id: str | None = None, shrink: int = 1) -> Self:
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

    @classmethod
    def from_pil(cls, pil_image: PIL.Image.Image, *, camera_id: str = 'from_pil', time: float | None = None) -> Self:
        """Create an image from a PIL image. This runs a JPEG encode."""
        bytesio = io.BytesIO()
        pil_image.save(bytesio, format='jpeg')
        size = ImageSize(width=pil_image.width, height=pil_image.height)
        return cls(camera_id=camera_id, size=size, time=time or rosys.time(), data=bytesio.getvalue())

    @classmethod
    def from_array(cls, array: np.ndarray, *, camera_id: str = 'from_array', time: float | None = None) -> Self:
        """Create an image from a NumPy array. This runs a JPEG encode."""
        _, encoded_image = cv2.imencode('.jpg', array)
        size = ImageSize(width=array.shape[1], height=array.shape[0])
        return cls(camera_id=camera_id, size=size, time=time or rosys.time(), data=encoded_image.tobytes())

    def to_array(self) -> np.ndarray:
        """Convert the image to a NumPy array. This runs a JPEG decode."""
        if self.data is None:
            raise ValueError('Cannot convert image to array because it has no data.')

        return cv2.imdecode(np.frombuffer(self.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def to_pil(self) -> PIL.Image.Image:
        """Convert the image to a PIL image. This runs a JPEG decode."""
        if self.data is None:
            raise ValueError('Cannot convert image to PIL image because it has no data.')

        return PIL.Image.open(io.BytesIO(self.data))
