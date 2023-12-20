from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np
from fastapi import Response
from nicegui import app

from .. import run
from .image import Image

if TYPE_CHECKING:
    from .camera import Camera

log = logging.getLogger('rosys.image_route')


def create_image_route(camera: Camera) -> None:
    async def get_camera_image(timestamp: str, shrink: int = 1) -> Response:
        return await _get_image(camera, timestamp, shrink)

    app.add_api_route('/' + camera.base_path + '/placeholder', _get_placeholder)
    app.add_api_route('/' + camera.base_path + '/{timestamp}', get_camera_image)


async def _get_placeholder(shrink: int = 1) -> Response:
    return Response(content=Image.create_placeholder('no image', shrink=shrink).data, media_type='image/jpeg')


async def _get_image(camera: Camera, timestamp: str, shrink: int = 1) -> Response:
    try:
        if not camera:
            return Response(content='Camera not found', status_code=404)
        jpeg = await _try_get_jpeg(camera, timestamp, shrink)
        if not jpeg:
            return Response(content='Image not found', status_code=404)
        return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
    except Exception:
        log.exception('could not get image')
        raise


async def _try_get_jpeg(camera: Camera, timestamp: str, shrink: int) -> Optional[bytes]:
    for image in reversed(camera.images):
        if str(image.time) == timestamp and image.data is not None:
            jpeg = image.data
            if shrink != 1:
                array = np.frombuffer(image.data, dtype=np.uint8)
                if array is None:
                    return None
                jpeg = await run.cpu_bound(_shrink, shrink, array)
            return jpeg
    return None


def _shrink(factor: int, array: np.ndarray) -> bytes:
    decoded = cv2.imdecode(array, cv2.IMREAD_COLOR)
    img = decoded[::factor, ::factor]
    _, encoded_image = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    return encoded_image.tobytes()
