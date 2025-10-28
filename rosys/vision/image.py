from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from typing import ClassVar, Self

import numpy as np
import PIL.Image
import PIL.ImageDraw

from rosys.vision.image_processing import decode_jpeg_image, encode_image_as_jpeg

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
    array: Image.array_type
    _detections: dict[str, Detections] = field(default_factory=dict)
    is_broken: bool | None = None
    tags: set[str] = field(default_factory=set)

    DEFAULT_PLACEHOLDER_SIZE: ClassVar[tuple[int, int]] = (320, 240)
    array_type = np.ndarray[tuple[int, int, int], np.dtype[np.uint8]]  # pixel data with shape (height, width, RGB)

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
        array = np.array(d)
        return cls(
            camera_id=camera_id or 'no_cam_id',
            time=time or 0,
            size=ImageSize(width=img.width, height=img.height),
            array=array,
        )

    @classmethod
    def from_pil(cls, pil_image: PIL.Image.Image, *, camera_id: str = 'from_pil', time: float | None = None) -> Self:
        """Create an image from a PIL image."""
        size = ImageSize(width=pil_image.width, height=pil_image.height)
        array = np.array(pil_image)
        return cls(camera_id=camera_id, size=size, time=time or rosys.time(), array=array)

    @classmethod
    def from_jpeg_bytes(cls, jpeg_bytes: bytes, *, camera_id: str = 'from_jpeg_bytes', time: float | None = None) -> Self:
        """Create an image from plain JPEG bytes. This runs a jpeg decode"""
        # TODO: what if decoding fails?
        array = decode_jpeg_image(jpeg_bytes)
        size = ImageSize(width=array.shape[1], height=array.shape[0])
        return cls(camera_id=camera_id, size=size, time=time or rosys.time(), array=array)

    @classmethod
    def from_array(cls, array: np.ndarray, *, camera_id: str = 'from_array', time: float | None = None) -> Self:
        """Create an image from a NumPy array."""
        assert array.dtype == np.uint8, "Array must have dtype np.uint8"
        assert len(array.shape) == 3, "Array must have shape (height, width, channels)"
        assert array.shape[2] == 3, "Image should have 3 channels"
        size = ImageSize(width=array.shape[1], height=array.shape[0])
        return cls(camera_id=camera_id, size=size, time=time or rosys.time(), array=array)

    def to_array(self) -> np.ndarray:
        """Create a copy of the internal pixel data"""  # TODO: remove?
        return np.copy(self.array)

    def to_pil(self) -> PIL.Image.Image:
        """Convert the image to a PIL image."""
        return PIL.Image.fromarray(self.array)

    def to_jpeg_bytes(self) -> bytes:
        """Convert the image to a JPEG image by encoding it."""
        return encode_image_as_jpeg(self.array)

    def byte_size(self) -> int:
        """Compute the size of the stored image representation in bytes."""
        return self.array.size
