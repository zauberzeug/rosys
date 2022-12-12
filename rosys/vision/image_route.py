import logging
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np
from fastapi import Response
from nicegui import app

from .. import run
from .camera import Camera
from .image import Image

if TYPE_CHECKING:
    from .camera_provider import CameraProvider

log = logging.getLogger('rosys.image_route')


def create_image_route(camera_provider: 'CameraProvider') -> None:
    async def get_image(camera_id: str, timestamp: str, shrink: int = 1) -> Response:
        return await _get_image(camera_provider.cameras, camera_id, timestamp, shrink)
    app.add_api_route('/' + camera_provider.base_path + '/{camera_id}/{timestamp}', get_image)
    app.add_api_route('/' + camera_provider.base_path + '/placeholder', _get_placeholder)


def _get_placeholder() -> Response:
    return Response(content=Image.create_placeholder('no image').data, media_type='image/jpeg')


async def _get_image(cameras: dict[str, Camera], camera_id: str, timestamp: str, shrink: int = 1) -> Response:
    try:
        camera = cameras.get(camera_id)
        if not camera:
            return Response(content='Camera not found', status_code=404)
        jpeg = await _try_get_jpeg(camera, timestamp, shrink)
        if not jpeg:
            return Response(content='Image not found', status_code=404)
        return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
    except:
        log.exception('could not get image')
        raise


async def _try_get_jpeg(camera: Camera, timestamp: str, shrink: int) -> Optional[bytes]:
    for image in reversed(camera.images):
        if str(image.time) == timestamp and image.data is not None:
            jpeg = image.data
            if shrink != 1:
                array = np.frombuffer(image.data, dtype=np.uint8)
                if array is None:
                    return
                jpeg = await run.cpu_bound(_shrink, shrink, array)
            return jpeg


def _shrink(factor: int, array: np.ndarray) -> bytes:
    decoded = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if decoded is None:
        return
    img = decoded[::factor, ::factor]
    return cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])[1].tobytes()
