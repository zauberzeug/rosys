from __future__ import annotations

from collections.abc import Iterable, Mapping
from types import MappingProxyType
from typing import ClassVar

import cv2
import numpy as np

from ..geometry import Polygon
from .image import Image, ImageArray


class ImageMasks:
    """Per-camera masks that gray out regions of an image.

    Each camera id maps to polygons in normalized image coordinates (x and y between 0 and 1),
    so the same polygons fit every resolution a camera may deliver.
    Pixels inside a camera's polygons are replaced by ``FILL_VALUE``, all others are kept;
    along the outlines, which are grayed out as well, the result is accurate to about a pixel.
    Polygons may overlap, and a polygon reaching 0 and 1 covers the full width or height of the image.
    A camera without polygons passes its images through unchanged.
    """

    FILL_VALUE: ClassVar[int] = 114
    """Gray value for the pixels inside the polygons.

    It matches the letterbox padding common detectors are trained with.
    """

    def __init__(self, polygons: Mapping[str, Iterable[Polygon]] | None = None) -> None:
        self._polygons: dict[str, tuple[Polygon, ...]] = {}
        for camera_id, camera_polygons in (polygons or {}).items():
            self.set(camera_id, camera_polygons)

    @property
    def polygons(self) -> Mapping[str, tuple[Polygon, ...]]:
        """A read-only view of the polygons per camera id."""
        return MappingProxyType(self._polygons)

    def __contains__(self, camera_id: object) -> bool:
        return camera_id in self._polygons

    def set(self, camera_id: str, polygons: Iterable[Polygon]) -> None:
        """Replace the polygons of a camera.

        :param camera_id: the camera the polygons apply to
        :param polygons: polygons with at least three points each and all coordinates between 0 and 1;
            without any polygons the camera passes its images through unchanged
        :raises ValueError: if a polygon has fewer than three points or a point lies outside the normalized range
        """
        camera_polygons = tuple(polygons)
        if not camera_polygons:
            self.remove(camera_id)
            return
        for polygon in camera_polygons:
            if len(polygon.outline) < 3:
                raise ValueError(f'Polygon for camera "{camera_id}" needs at least three points, '
                                 f'got {len(polygon.outline)}')
            for point in polygon.outline:
                if not (0.0 <= point.x <= 1.0 and 0.0 <= point.y <= 1.0):
                    raise ValueError(f'Polygon for camera "{camera_id}" has a point outside the normalized range: '
                                     f'{point}')
        self._polygons[camera_id] = camera_polygons

    def remove(self, camera_id: str) -> None:
        """Remove the polygons of a camera so its images pass through unchanged."""
        self._polygons.pop(camera_id, None)

    def apply(self, image: Image) -> ImageArray:
        """Return the pixels of an image with everything inside the camera's polygons replaced by ``FILL_VALUE``.

        The image itself is left untouched.
        If the camera has no polygons, the image's own array is returned without a copy.
        """
        polygons = self._polygons.get(image.camera_id)
        if polygons is None:
            return image.array
        masked: ImageArray = np.frombuffer(self._mask(image.array, polygons), np.uint8).reshape(image.array.shape)
        return masked

    def apply_as_bytes(self, image: Image) -> bytes | bytearray:
        """Return the masked pixels in C order, like ``apply(image).tobytes()`` but without copying them twice."""
        polygons = self._polygons.get(image.camera_id)
        if polygons is None:
            return image.array.tobytes()
        return self._mask(image.array, polygons)

    def _mask(self, array: ImageArray, polygons: tuple[Polygon, ...]) -> bytearray:
        buffer = bytearray(array.data)
        masked = np.frombuffer(buffer, np.uint8).reshape(array.shape)
        height, width = array.shape[:2]
        for polygon in polygons:  # one call per polygon, because a single call would leave overlaps unfilled
            cv2.fillPoly(masked, [_contour(polygon, width, height)], (self.FILL_VALUE,) * 3, shift=_FRACTIONAL_BITS)
        return buffer


_FRACTIONAL_BITS = 8


def _contour(polygon: Polygon, width: int, height: int) -> np.ndarray:
    """Convert a polygon in normalized coordinates to fixed-point pixel coordinates for OpenCV.

    Normalized 0 and 1 are the outer edges of the image, while OpenCV places pixel centers at integer coordinates.
    """
    points = np.array([[point.x * width, point.y * height] for point in polygon.outline]) - 0.5
    return np.round(points * (1 << _FRACTIONAL_BITS)).astype(np.int32)
