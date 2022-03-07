from functools import lru_cache
from typing import Optional
from nicegui.ui import Ui
from starlette.requests import Request
from starlette.responses import JSONResponse, Response
from starlette.routing import Route
from rosys import Runtime
import logging
import cv2
import numpy as np

log = logging.getLogger('rosys.routes')


def setup(ui: Ui, runtime: Runtime):
    ui.add_route(Route('/world', lambda request, **_: Response(content=runtime.world.json(), media_type='text/json')))
    ui.add_route(Route('/export', lambda request, **_: JSONResponse(content=runtime.persistence.dump())))

    @lru_cache(maxsize=100)
    def try_get_jpeg(cam_id: str, timestamp: str, shrink: int) -> Optional[bytes]:
        camera = runtime.world.cameras.get(cam_id)
        if camera is None:
            return
        for image in reversed(camera.images):
            if str(image.time) == timestamp:
                jpeg = image.data
                if shrink != 1:
                    array = np.frombuffer(image.data, dtype=np.uint8)
                    img = cv2.imdecode(array, cv2.IMREAD_COLOR)[::shrink, ::shrink]
                    jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1].tobytes()
                return jpeg

    def get_image(request: Request, **_):
        try:
            jpeg = try_get_jpeg(request.path_params['id'],
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
