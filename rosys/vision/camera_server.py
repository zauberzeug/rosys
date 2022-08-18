import logging
from typing import Optional

import cv2
import numpy as np
from aiocache import cached
from nicegui import ui
from starlette.requests import Request
from starlette.responses import Response
from starlette.routing import Route

from .. import run
from .camera_provider import CameraProvider
from .image import Image


def shrink(factor: int, array: np.ndarray) -> bytes:
    img = cv2.imdecode(array, cv2.IMREAD_COLOR)[::factor, ::factor]
    return cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])[1].tobytes()


class CameraServer:
    '''A camera server creates an HTTP route to access images of a given camera provider.'''

    def __init__(self, camera_module: CameraProvider) -> None:
        self.log = logging.getLogger('rosys.camera_server')
        self.camera_module = camera_module

        ui.add_route(Route('/camera/{id}/{timestamp}', self.get_image))
        ui.add_route(Route('/camera/placeholder', lambda _: Response(
            content=Image.create_placeholder('no image').data,
            media_type='image/jpeg'
        )))

    @cached(ttl=30)
    async def try_get_jpeg(self, cam_id: str, timestamp: str, shrink_factor: int) -> Optional[bytes]:
        camera = self.camera_module.cameras.get(cam_id)
        if camera is None:
            return
        for image in reversed(camera.images):
            if str(image.time) == timestamp:
                jpeg = image.data
                if shrink_factor != 1:
                    array = np.frombuffer(image.data, dtype=np.uint8)
                    jpeg = await run.cpu_bound(shrink, shrink_factor, array)
                return jpeg

    async def get_image(self, request: Request, **_) -> None:
        try:
            jpeg = await self.try_get_jpeg(request.path_params['id'],
                                           request.path_params['timestamp'],
                                           int(request.query_params.get('shrink', 1)))
            if jpeg:
                return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
            else:
                return Response(content='Not Found', status_code=404)
        except:
            self.log.exception('could not get image')
            raise
