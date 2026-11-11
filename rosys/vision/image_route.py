from __future__ import annotations

import asyncio
import logging
import math
import os
import weakref
from collections import OrderedDict
from typing import TYPE_CHECKING

import cv2
import numpy as np
from fastapi import Response
from nicegui import app

from .. import run
from ..geometry import Rectangle
from .calibration import Calibration
from .image import Image
from .image_processing import DEFAULT_JPEG_QUALITY, encode_image_as_jpeg, process_ndarray_image, validate_jpeg_quality
from .image_rotation import ImageRotation

if TYPE_CHECKING:
    from .camera import Camera

log = logging.getLogger('rosys.image_route')

MAX_CONCURRENT_ENCODINGS: int = max(1, (getattr(os, 'process_cpu_count', os.cpu_count)() or 4) - 1)
"""Images encoded at the same time: one worker of the process pool (one per CPU) stays free for its other jobs;
further requests wait without holding a frame, so a burst cannot pile up frames."""
JPEG_CACHE_SIZE = 16
"""Encoded images kept for other clients that ask for the same frame with the same parameters."""

_route_owners: dict[str, weakref.ref[Camera]] = {}

JpegKey = tuple[object, ...]


class _Encoding:
    """The encoding slots and the cache of the image routes."""

    def __init__(self) -> None:
        self.slots = asyncio.Semaphore(MAX_CONCURRENT_ENCODINGS)
        self.cache: OrderedDict[JpegKey, bytes] = OrderedDict()
        self.pending: dict[JpegKey, asyncio.Future[bytes | None]] = {}

    def cached(self, key: JpegKey) -> bytes | None:
        jpeg = self.cache.get(key)
        if jpeg is not None:
            self.cache.move_to_end(key)
        return jpeg

    def remember(self, key: JpegKey, jpeg: bytes) -> None:
        self.cache[key] = jpeg
        while len(self.cache) > JPEG_CACHE_SIZE:
            self.cache.popitem(last=False)


_encoding = _Encoding()


def reset_encodings() -> None:
    """Free the encoding slots and empty the cache, e.g. between tests.

    Every test runs its own event loop and starts at time 0, while the slots bind to the first loop that waits on them
    and the cache would answer with the image another test encoded for the same timestamp.
    """
    _encoding.slots = asyncio.Semaphore(MAX_CONCURRENT_ENCODINGS)
    _encoding.cache.clear()
    _encoding.pending.clear()


def create_image_route(camera: Camera) -> None:
    placeholder_url = '/' + camera.base_path + '/placeholder'
    timestamp_url = '/' + camera.base_path + '/{timestamp}'
    undistorted_url = '/' + camera.base_path + '/{timestamp}/undistorted'

    app.remove_route(placeholder_url)
    app.remove_route(timestamp_url)
    app.remove_route(undistorted_url)

    camera_ref = weakref.ref(camera)
    urls = (placeholder_url, timestamp_url, undistorted_url)
    for url in urls:
        _route_owners[url] = camera_ref  # NOTE: claim the urls before adding, so a pending finalizer cannot remove them

    async def get_camera_image(timestamp: str,
                               shrink: float = 1.0,
                               max_dimension: float | None = None,
                               fast: bool = True,
                               compression: int = DEFAULT_JPEG_QUALITY,
                               crop_x: int | None = None,
                               crop_y: int | None = None,
                               crop_w: int | None = None,
                               crop_h: int | None = None,
                               rotation: int = 0) -> Response:
        camera = camera_ref()
        if not camera:
            return Response(content='Camera was removed', status_code=404)
        try:
            validate_jpeg_quality(compression)
            crop = Rectangle(x=crop_x, y=crop_y, width=crop_w, height=crop_h) \
                if crop_x is not None and crop_y is not None and crop_w is not None and crop_h is not None else None
            image_rotation = ImageRotation.from_degrees(rotation)
        except ValueError as e:
            return Response(content=str(e), status_code=400)
        return await _get_image(camera, timestamp,
                                shrink=shrink,
                                max_dimension=max_dimension,
                                undistort=False,
                                fast=fast,
                                compression=compression,
                                crop=crop,
                                rotation=image_rotation)

    async def get_camera_image_undistorted(timestamp: str,
                                           shrink: float = 1.0,
                                           max_dimension: float | None = None,
                                           fast: bool = True,
                                           compression: int = DEFAULT_JPEG_QUALITY,
                                           crop_x: int | None = None,
                                           crop_y: int | None = None,
                                           crop_w: int | None = None,
                                           crop_h: int | None = None,
                                           rotation: int = 0) -> Response:
        camera = camera_ref()
        if not camera:
            return Response(content='Camera was removed', status_code=404)
        try:
            validate_jpeg_quality(compression)
            crop = Rectangle(x=crop_x, y=crop_y, width=crop_w, height=crop_h) \
                if crop_x is not None and crop_y is not None and crop_w is not None and crop_h is not None else None
            image_rotation = ImageRotation.from_degrees(rotation)
        except ValueError as e:
            return Response(content=str(e), status_code=400)
        return await _get_image(camera,
                                timestamp,
                                shrink=shrink,
                                max_dimension=max_dimension,
                                undistort=True,
                                fast=fast,
                                compression=compression,
                                crop=crop,
                                rotation=image_rotation)

    app.add_api_route(placeholder_url, _get_placeholder)
    app.add_api_route(timestamp_url, get_camera_image)
    app.add_api_route(undistorted_url, get_camera_image_undistorted)

    weakref.finalize(camera, _remove_image_routes, camera_ref, urls)


def _remove_image_routes(camera_ref: weakref.ref[Camera], urls: tuple[str, ...]) -> None:
    for url in urls:
        if _route_owners.get(url) is camera_ref:  # NOTE: a newer camera with the same id may own them by now
            app.remove_route(url)
            del _route_owners[url]


def _create_placeholder(shrink: int) -> bytes:
    image = Image.create_placeholder('no image', shrink=shrink)
    return image.to_jpeg_bytes()


async def _get_placeholder(shrink: int = 1) -> Response:
    placeholder_jpeg = await run.cpu_bound(_create_placeholder, shrink)
    if placeholder_jpeg is None:
        return Response('Server is stopping', 500)
    return Response(placeholder_jpeg, media_type='image/jpeg')


async def _get_image(camera: Camera,
                     timestamp: str, *,
                     shrink: float,
                     max_dimension: float | None,
                     undistort: bool,
                     fast: bool,
                     compression: int,
                     crop: Rectangle | None = None,
                     rotation: ImageRotation = ImageRotation.NONE) -> Response:
    try:
        if undistort and getattr(camera, 'calibration', None) is None:
            return Response(content='Camera is not calibrated', status_code=404)
        jpeg = await _try_get_jpeg(camera,
                                   timestamp,
                                   shrink=shrink,
                                   max_dimension=max_dimension,
                                   undistort=undistort,
                                   fast=fast,
                                   compression=compression,
                                   crop=crop,
                                   rotation=rotation)
        if not jpeg:
            return Response(content='Image not found', status_code=404)
        return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
    except Exception:
        log.exception('could not get image')
        raise


async def _try_get_jpeg(camera: Camera,
                        timestamp: str, *,
                        shrink: float,
                        max_dimension: float | None,
                        undistort: bool,
                        fast: bool,
                        compression: int,
                        crop: Rectangle | None = None,
                        rotation: ImageRotation = ImageRotation.NONE) -> bytes | None:
    """Encode the camera's frame of ``timestamp``, at most ``MAX_CONCURRENT_ENCODINGS`` at a time.

    A request looks up its frame only once it holds a slot, so waiting requests keep no frame alive,
    and a frame that left the camera's buffer is answered with ``None`` without waiting for a slot.
    Clients asking for the same frame with the same parameters share one encoding and its cached result.
    """
    key: JpegKey = (camera.base_path, timestamp, shrink, max_dimension, undistort, fast, compression,
                    None if crop is None else (crop.x, crop.y, crop.width, crop.height), rotation)
    encoding = _encoding
    while True:
        jpeg = encoding.cached(key)
        if jpeg is not None:
            return jpeg
        if not any(str(image.time) == timestamp for image in camera.images):
            return None
        pending = encoding.pending.get(key)
        if pending is None:
            break
        jpeg = await asyncio.shield(pending)
        if jpeg is not None:
            return jpeg
        # NOTE: the request that encoded this image got none, e.g. it was cancelled, so try again
    future: asyncio.Future[bytes | None] = asyncio.get_running_loop().create_future()
    encoding.pending[key] = future
    jpeg = None
    try:
        async with encoding.slots:
            jpeg = await _encode_frame(camera, timestamp, shrink=shrink, max_dimension=max_dimension,
                                       undistort=undistort, fast=fast, compression=compression,
                                       crop=crop, rotation=rotation)
        if jpeg is not None:
            encoding.remember(key, jpeg)
        return jpeg
    finally:
        del encoding.pending[key]
        future.set_result(jpeg)


async def _encode_frame(camera: Camera,
                        timestamp: str, *,
                        shrink: float,
                        max_dimension: float | None,
                        undistort: bool,
                        fast: bool,
                        compression: int,
                        crop: Rectangle | None,
                        rotation: ImageRotation) -> bytes | None:
    for image in reversed(camera.images):
        if str(image.time) == timestamp:
            shrink_from_max = max(image.size.width, image.size.height) / max_dimension if max_dimension else shrink

            shrink = max(1, shrink, shrink_from_max)

            if not undistort and crop is None and rotation == ImageRotation.NONE and (shrink == 1 or fast):
                # NOTE: a view of every step-th pixel pickles only those pixels, so the worker receives what it encodes
                step = math.ceil(shrink)
                return await run.cpu_bound(encode_image_as_jpeg, image.array[::step, ::step], compression)

            calibration = camera.calibration if undistort else None  # type: ignore
            return await run.cpu_bound(_process, image, calibration, shrink, undistort, fast, compression,
                                       crop, rotation)

    return None


def _process(image: Image,
             calibration: Calibration | None,
             shrink: float,
             undistort: bool,
             fast: bool,
             compression: int,
             crop: Rectangle | None = None,
             rotation: ImageRotation = ImageRotation.NONE) -> bytes | None:
    image_array: np.ndarray = image.array

    if undistort:
        assert calibration is not None
        image_array = calibration.undistort_image(image_array, crop=True)
        if image_array is None or image_array.size == 0:
            logging.warning('undistort_array returned an empty image')
            return None

    if crop is not None or rotation != ImageRotation.NONE:
        image_array = process_ndarray_image(image_array, rotation, crop)

    if shrink != 1.0:
        if fast:
            shrink_factor = math.ceil(shrink)
            image_array = image_array[::shrink_factor, ::shrink_factor]
        else:
            new_width = int(image_array.shape[1] / shrink)
            new_height = int(image_array.shape[0] / shrink)
            # INTER_AREA is optimal for downsampling
            image_array = cv2.resize(image_array, (new_width, new_height), interpolation=cv2.INTER_AREA)

    return encode_image_as_jpeg(image_array, quality=compression)
