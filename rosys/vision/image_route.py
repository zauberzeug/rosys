from __future__ import annotations

import logging
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

    async def get_camera_image(timestamp: str, shrink: int = 1) -> Response:
        return await _get_image(camera, timestamp, shrink=shrink)

    async def get_camera_image_undistorted(timestamp: str, shrink: int = 1) -> Response:
        return await _get_image(camera, timestamp, shrink=shrink, undistort=True)

    app.add_api_route(placeholder_url, _get_placeholder)
    app.add_api_route(timestamp_url, get_camera_image)
    app.add_api_route(undistorted_url, get_camera_image_undistorted)


async def _get_placeholder(shrink: int = 1) -> Response:
    return Response(content=Image.create_placeholder('no image', shrink=shrink).data, media_type='image/jpeg')


async def _get_image(camera: Camera, timestamp: str, *, shrink: int = 1, undistort: bool = False) -> Response:
    try:
        if not camera:
            return Response(content='Camera not found', status_code=404)
        if undistort and getattr(camera, 'calibration', None) is None:
            return Response(content='Camera is not calibrated', status_code=404)
        jpeg = await _try_get_jpeg(camera, timestamp, shrink=shrink, undistort=undistort)
        if not jpeg:
            return Response(content='Image not found', status_code=404)
        return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
    except Exception:
        log.exception('could not get image')
        raise


async def _try_get_jpeg(camera: Camera, timestamp: str, *, shrink: int, undistort: bool) -> bytes | None:
    for image in reversed(camera.images):
        if str(image.time) == timestamp and image.data is not None:
            if shrink == 1 and not undistort:
                return image.data
            else:
                calibration = camera.calibration if undistort else None  # type: ignore
                return await run.cpu_bound(_process, image.data, calibration, shrink, undistort)
    return None


def _process(data: bytes, calibration: Calibration | None, shrink: int, undistort: bool) -> bytes | None:
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

    if shrink != 1:
        image_array = image_array[::shrink, ::shrink]

    _, encoded_image = cv2.imencode('.jpg', image_array, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    return encoded_image.tobytes()
