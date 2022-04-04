import logging
from typing import Optional

import cv2
import numpy as np
import rosys
from aiocache import cached
from nicegui.ui import Ui
from starlette.requests import Request
from starlette.responses import JSONResponse, Response
from starlette.routing import Route

log = logging.getLogger('rosys.routes')


def shrink(factor: int, array: np.ndarray) -> bytes:
    img = cv2.imdecode(array, cv2.IMREAD_COLOR)[::factor, ::factor]
    return cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])[1].tobytes()


def setup(ui: Ui, runtime: rosys.Runtime):
    ui.add_route(Route('/world', lambda request, **_: Response(content=runtime.world.json(), media_type='text/json')))
    ui.add_route(Route('/export', lambda request, **_: JSONResponse(content=runtime.persistence.dump())))

    @cached(ttl=30)
    async def try_get_jpeg(cam_id: str, timestamp: str, shrink_factor: int) -> Optional[bytes]:
        camera = runtime.world.cameras.get(cam_id)
        if camera is None:
            return
        for image in reversed(camera.images):
            if str(image.time) == timestamp:
                jpeg = image.data
                if shrink_factor != 1:
                    array = np.frombuffer(image.data, dtype=np.uint8)
                    jpeg = await rosys.run.cpu_bound(shrink, shrink_factor, array)
                return jpeg

    async def get_image(request: Request, **_):
        try:
            jpeg = await try_get_jpeg(request.path_params['id'],
                                      request.path_params['timestamp'],
                                      int(request.query_params.get('shrink', 1)))
            if jpeg:
                return Response(content=jpeg, headers={'cache-control': 'max-age=7776000'}, media_type='image/jpeg')
            else:
                return Response(content='Not Found', status_code=404)
        except:
            log.exception('could not get image')
            raise

    ui.add_route(Route('/camera/{id}/{timestamp}', get_image))
    ui.add_route(Route('/camera/placeholder', lambda _: Response(
        content=rosys.world.Image.create_placeholder('no image'),
        media_type='image/jpeg'
    )))
