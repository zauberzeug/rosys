from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, ClassVar, Self

import numpy as np
import PIL.Image
import PIL.ImageDraw

from .. import rosys
from .detections import Detections
from .image_processing import decode_jpeg_image, encode_image_as_jpeg


@dataclass(slots=True, kw_only=True)
class ImageSize:
    width: int
    height: int

    @property
    def tuple(self) -> tuple[int, int]:
        return (self.width, self.height)


ImageArray = np.ndarray[tuple[int, int, int], np.dtype[np.uint8]]
'''An RGB NumPy array with shape (height, width, 3) and dtype np.uint8.'''


@dataclass(slots=True, kw_only=True)
class Image:
    camera_id: str
    time: float  # time of recording
    _array: ImageArray
    _detections: dict[str, Detections] = field(default_factory=dict)
    metadata: dict[str, Any]

    DEFAULT_PLACEHOLDER_SIZE: ClassVar[tuple[int, int]] = (320, 240)

    @property
    def detections(self) -> Detections | None:
        if not self._detections:
            return None
        if len(self._detections) > 1:
            raise RuntimeError(
                f'Image has multiple detection types ({", ".join(self._detections.keys())}). Use `get_detections(type)` instead.')
        return next(iter(self._detections.values()))

    @property
    def array(self) -> ImageArray:
        return self._array

    @property
    def size(self) -> ImageSize:
        return ImageSize(width=self._array.shape[1], height=self._array.shape[0])

    def get_detections(self, detector_id: str) -> Detections | None:
        return self._detections.get(detector_id)

    def set_detections(self, detector_id: str, detections: Detections) -> None:
        self._detections[detector_id] = detections

    @property
    def id(self) -> str:
        return f'{self.camera_id}/{self.time}'

    @classmethod
    def create_placeholder(cls, text: str, *,
                           camera_id: str = 'from_placeholder',
                           time: float | None = None,
                           metadata: dict[str, Any] | None = None,
                           shrink: int = 1) -> Self:
        h, w = cls.DEFAULT_PLACEHOLDER_SIZE
        img = PIL.Image.new('RGB', (h // shrink, w // shrink), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width / 2 - len(text) * 3, img.height / 2 - 5), text, fill=(255, 255, 255))
        return cls.from_array(np.array(img), camera_id=camera_id, time=time, metadata=metadata)

    @classmethod
    def from_pil(cls, pil_image: PIL.Image.Image, *,
                 camera_id: str = 'from_pil',
                 time: float | None = None,
                 metadata: dict[str, Any] | None = None) -> Self:
        """Create an image from a PIL image."""
        return cls.from_array(np.array(pil_image), camera_id=camera_id, time=time, metadata=metadata)

    @classmethod
    def from_jpeg_bytes(cls, jpeg_bytes: bytes, *,
                        camera_id: str = 'from_jpeg_bytes',
                        time: float | None = None,
                        metadata: dict[str, Any] | None = None) -> Self | None:
        """Create an image from plain JPEG bytes. This runs a jpeg decode. Returns None if decoding fails."""
        array = decode_jpeg_image(jpeg_bytes)
        if array is None:
            return None
        if len(array.shape) == 2:
            array = np.repeat(np.expand_dims(array, -1), repeats=3, axis=2)
        return cls.from_array(array, camera_id=camera_id, time=time, metadata=metadata)

    @classmethod
    def from_array(cls, array: ImageArray, *,
                   camera_id: str = 'from_array',
                   time: float | None = None,
                   metadata: dict[str, Any] | None = None) -> Self:
        """Create an image from a NumPy array."""

        # Convert array to a c-contiguous representation (or do nothing if that
        # is already the case). This ensures fast pickling.
        array = np.ascontiguousarray(array)

        assert array.dtype == np.uint8, 'Array must have dtype np.uint8'
        assert len(array.shape) == 3, 'Array must have shape (height, width, channels)'
        assert array.shape[2] == 3, 'Image should have 3 channels'
        return cls(camera_id=camera_id, time=time or rosys.time(), _array=array, metadata=metadata or {})

    def to_pil(self) -> PIL.Image.Image:
        """Convert the image to a PIL image."""
        return PIL.Image.fromarray(self.array)

    def to_jpeg_bytes(self) -> bytes:
        """Convert the image to a JPEG image by encoding it."""
        return encode_image_as_jpeg(self.array)

    def byte_size(self) -> int:
        """Compute the size of the stored image representation in bytes."""
        return self.array.size
