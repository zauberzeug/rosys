from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import cv2
import numpy as np
from fastapi import Response
from nicegui import app

from .. import run
from .calibration import Calibration
from .image import Image

if TYPE_CHECKING:
    from .camera import Camera

log = logging.getLogger('rosys.image_route')


def create_image_route(camera: Camera) -> None:
    placeholder_url = '/' + camera.base_path + '/placeholder'
    timestamp_url = '/' + camera.base_path + '/{timestamp}'
    undistorted_url = '/' + camera.base_path + '/{timestamp}/undistorted'

    app.remove_route(placeholder_url)
    app.remove_route(timestamp_url)
    app.remove_route(undistorted_url)

    async def get_camera_image(timestamp: str,
                               shrink: float = 1.0,
                               max_dimension: float | None = None,
                               fast: bool = True,
                               compression: int = 60) -> Response:
        return await _get_image(camera, timestamp,
                                shrink=shrink,
                                max_dimension=max_dimension,
                                undistort=False,
                                fast=fast,
                                compression=compression)

    async def get_camera_image_undistorted(timestamp: str,
                                           shrink: float = 1.0,
                                           max_dimension: float | None = None,
                                           fast: bool = True,
                                           compression: int = 60) -> Response:
        return await _get_image(camera,
                                timestamp,
                                shrink=shrink,
                                max_dimension=max_dimension,
                                undistort=True,
                                fast=fast,
                                compression=compression)

    app.add_api_route(placeholder_url, _get_placeholder)
    app.add_api_route(timestamp_url, get_camera_image)
    app.add_api_route(undistorted_url, get_camera_image_undistorted)


async def _get_placeholder(shrink: int = 1) -> Response:
    return Response(content=Image.create_placeholder('no image', shrink=shrink).data, media_type='image/jpeg')


async def _get_image(camera: Camera,
                     timestamp: str, *,
                     shrink: float,
                     max_dimension: float | None,
                     undistort: bool,
                     fast: bool,
                     compression: int) -> Response:
    try:
        if undistort and getattr(camera, 'calibration', None) is None:
            return Response(content='Camera is not calibrated', status_code=404)
        jpeg = await _try_get_jpeg(camera,
                                   timestamp,
                                   shrink=shrink,
                                   max_dimension=max_dimension,
                                   undistort=undistort,
                                   fast=fast,
                                   compression=compression)
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
                        compression: int) -> bytes | None:
    for image in reversed(camera.images):
        if str(image.time) == timestamp and image.data is not None:
            shrink_from_max = max(image.size.width, image.size.height) / max_dimension if max_dimension else shrink

            shrink = max(1, shrink, shrink_from_max)

            if shrink == 1 and not undistort and compression == 60:
                return image.data

            calibration = camera.calibration if undistort else None  # type: ignore
            return await run.cpu_bound(_process, image.data, calibration, shrink, undistort, fast, compression)

    return None


def _process(data: bytes,
             calibration: Calibration | None,
             shrink: float,
             undistort: bool,
             fast: bool,
             compression: int) -> bytes | None:
    array = np.frombuffer(data, dtype=np.uint8)
    if array is None:
        return None
    image_array = cv2.imdecode(array, cv2.IMREAD_COLOR)

    if undistort:
        assert calibration is not None
        image_array = calibration.undistort_array(image_array, crop=True)
        if image_array is None or image_array.size == 0:
            logging.warning('undistort_array returned an empty image')
            return None

    if shrink != 1.0:
        if fast:
            shrink_factor = math.ceil(shrink)
            image_array = image_array[::shrink_factor, ::shrink_factor]
        else:
            new_width = int(image_array.shape[1] / shrink)
            new_height = int(image_array.shape[0] / shrink)
            # INTER_AREA is optimal for downsampling
            image_array = cv2.resize(image_array, (new_width, new_height), interpolation=cv2.INTER_AREA)

    _, encoded_image = cv2.imencode('.jpg', image_array, [int(cv2.IMWRITE_JPEG_QUALITY), compression])
    return encoded_image.tobytes()
