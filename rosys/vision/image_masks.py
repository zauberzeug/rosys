from __future__ import annotations

import functools
from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import ClassVar

import cv2
import numpy as np

from ..geometry import Polygon
from .image import Image, ImageArray


class ImageMasks:
    """Per-camera regions of interest for detectors.

    Each camera id maps to polygons in normalized image coordinates (x and y between 0 and 1),
    so the same polygons fit every resolution a camera may deliver.
    Pixels inside a camera's polygons or on their outlines are kept, all others are replaced by ``FILL_VALUE``.
    Polygons may overlap, and a polygon reaching 0 and 1 keeps the full width or height of the image.
    A camera without polygons passes its images through unchanged.
    """

    FILL_VALUE: ClassVar[int] = 114
    """Gray value for the pixels outside the kept region.

    It matches the letterbox padding common detectors are trained with.
    """

    def __init__(self, polygons: Mapping[str, Sequence[Polygon]] | None = None) -> None:
        self._polygons: dict[str, tuple[Polygon, ...]] = {}
        for camera_id, camera_polygons in (polygons or {}).items():
            self.set(camera_id, camera_polygons)

    @property
    def polygons(self) -> Mapping[str, tuple[Polygon, ...]]:
        """A read-only view of the polygons per camera id."""
        return MappingProxyType(self._polygons)

    def __contains__(self, camera_id: object) -> bool:
        return camera_id in self._polygons

    def set(self, camera_id: str, polygons: Sequence[Polygon]) -> None:
        """Replace the polygons of a camera.

        :param camera_id: the camera the polygons apply to
        :param polygons: at least one polygon with at least three points, all coordinates between 0 and 1
        :raises ValueError: if the polygons are empty, degenerate or outside the normalized range
        """
        if not polygons:
            raise ValueError(f'At least one polygon is required for camera "{camera_id}"')
        for polygon in polygons:
            if len(polygon.outline) < 3:
                raise ValueError(f'Polygon for camera "{camera_id}" needs at least three points, '
                                 f'got {len(polygon.outline)}')
            for point in polygon.outline:
                if not (0.0 <= point.x <= 1.0 and 0.0 <= point.y <= 1.0):
                    raise ValueError(f'Polygon for camera "{camera_id}" has a point outside the normalized range: '
                                     f'{point}')
        self._polygons[camera_id] = tuple(polygons)

    def remove(self, camera_id: str) -> None:
        """Remove the polygons of a camera so its images pass through unchanged."""
        self._polygons.pop(camera_id, None)

    def apply(self, image: Image) -> ImageArray:
        """Return the pixels of an image with everything outside the camera's polygons replaced by ``FILL_VALUE``.

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
        buffer = bytearray([self.FILL_VALUE]) * array.nbytes
        masked = np.frombuffer(buffer, np.uint8).reshape(array.shape)
        height, width = array.shape[:2]
        for polygon in polygons:
            raster = _raster(width, height, _contour(polygon, width, height).tobytes())
            if raster is None:
                continue
            left, top, mask = raster
            bottom, right = top + mask.shape[0], left + mask.shape[1]
            cv2.copyTo(array[top:bottom, left:right], mask, masked[top:bottom, left:right])
        return buffer


def _contour(polygon: Polygon, width: int, height: int) -> np.ndarray:
    """Convert a polygon in normalized coordinates to integer pixel coordinates for OpenCV."""
    points = np.array([[point.x * width, point.y * height] for point in polygon.outline])
    return np.round(points).astype(np.int32)


@functools.lru_cache(maxsize=16)
def _raster(width: int, height: int, contour: bytes) -> tuple[int, int, np.ndarray] | None:
    """Rasterize a contour within its bounding box clipped to the image, and return the box's left and top with it."""
    polygon = np.frombuffer(contour, np.int32).reshape(-1, 2)
    x, y, w, h = cv2.boundingRect(polygon)
    left, top, right, bottom = x, y, min(x + w, width), min(y + h, height)
    if left >= right or top >= bottom:
        return None
    mask: np.ndarray = np.zeros((bottom - top, right - left), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255, offset=(-left, -top))
    return left, top, mask
