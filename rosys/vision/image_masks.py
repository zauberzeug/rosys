from __future__ import annotations

from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import ClassVar

import cv2
import numpy as np

from ..geometry import Polygon
from .image import Image, ImageArray, ImageSize

Raster = np.ndarray[tuple[int, int], np.dtype[np.uint8]]


class ImageMasks:
    """Per-camera regions of interest for detectors.

    Each camera id maps to polygons in normalized image coordinates (x and y between 0 and 1),
    so the same polygons fit every resolution a camera may deliver.
    Pixels inside any of a camera's polygons are kept, all other pixels are replaced by ``FILL_VALUE``.
    A camera without polygons passes its images through unchanged.

    The rasterized mask is cached per camera for the last image size seen,
    so sending images of changing size from the same camera rasterizes on every call.
    """

    FILL_VALUE: ClassVar[int] = 114
    """Gray value for the pixels outside the kept region.

    It matches the letterbox padding common detectors are trained with.
    """

    def __init__(self, polygons: Mapping[str, Sequence[Polygon]] | None = None) -> None:
        self._polygons: dict[str, tuple[Polygon, ...]] = {}
        self._rasters: dict[str, tuple[int, int, Raster]] = {}
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
        self._rasters.pop(camera_id, None)

    def remove(self, camera_id: str) -> None:
        """Remove the polygons of a camera so its images pass through unchanged."""
        self._polygons.pop(camera_id, None)
        self._rasters.pop(camera_id, None)

    def apply(self, image: Image) -> ImageArray:
        """Return the pixels of an image with everything outside the camera's polygons replaced by ``FILL_VALUE``.

        The image itself is left untouched.
        If the camera has no polygons, the image's own array is returned without a copy.
        """
        if image.camera_id not in self._polygons:
            return image.array
        raster = self._raster(image.camera_id, image.size)
        masked: ImageArray = np.full_like(image.array, self.FILL_VALUE)
        cv2.copyTo(image.array, raster, masked)
        return masked

    def _raster(self, camera_id: str, size: ImageSize) -> Raster:
        cached = self._rasters.get(camera_id)
        if cached is not None and cached[0] == size.width and cached[1] == size.height:
            return cached[2]
        raster: Raster = np.zeros((size.height, size.width), dtype=np.uint8)
        contours: list[np.ndarray] = [
            np.round([[point.x * size.width, point.y * size.height] for point in polygon.outline]).astype(np.int32)
            for polygon in self._polygons[camera_id]
        ]
        cv2.fillPoly(raster, contours, 255)
        self._rasters[camera_id] = (size.width, size.height, raster)
        return raster
