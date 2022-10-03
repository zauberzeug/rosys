import logging
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np
from nicegui import ui
from starlette.requests import Request
from starlette.responses import Response
from starlette.routing import Route

from .. import run
from .camera import Camera
from .image import Image

if TYPE_CHECKING:
    from .camera_provider import CameraProvider

log = logging.getLogger('rosys.image_route')


def create_image_route(camera_provider: 'CameraProvider') -> None:
    async def get_image(request: Request, **_) -> Response:
        return await _get_image(camera_provider.cameras, request)
    ui.add_route(Route('/' + camera_provider.base_path + '/{camera_id}/{timestamp}', get_image))
    ui.add_route(Route('/' + camera_provider.base_path + '/placeholder', _get_placeholder))


def _get_placeholder(_) -> Response:
    return Response(content=Image.create_placeholder('no image').data, media_type='image/jpeg')


async def _get_image(cameras: dict[str, Camera], request: Request) -> None:
    try:
        camera = cameras.get(request.path_params['camera_id'])
        if not camera:
            return Response(content='Camera not found', status_code=404)

        shrink_factor = int(request.query_params.get('shrink', 1))
        jpeg = await _try_get_jpeg(camera, request.path_params['timestamp'], shrink_factor)
        if not jpeg:
            return Response(content='Image not found', status_code=404)

        return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
    except:
        log.exception('could not get image')
        raise


async def _try_get_jpeg(camera: Camera, timestamp: str, shrink_factor: int) -> Optional[bytes]:
    for image in reversed(camera.images):
        if str(image.time) == timestamp and image.data is not None:
            jpeg = image.data
            if shrink_factor != 1:
                array = np.frombuffer(image.data, dtype=np.uint8)
                if array is None:
                    return
                jpeg = await run.cpu_bound(_shrink, shrink_factor, array)
            return jpeg


def _shrink(factor: int, array: np.ndarray) -> bytes:
    decoded = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if decoded is None:
        return
    img = decoded[::factor, ::factor]
    return cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])[1].tobytes()
