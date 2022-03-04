from nicegui.ui import Ui
from starlette.requests import Request
from starlette.responses import JSONResponse, Response
from starlette.routing import Route
from rosys import Runtime
import logging
import cv2
import numpy as np

log = logging.getLogger('rosys.routes')
not_found = Response(content='Not Found', status_code=404)


def setup(ui: Ui, runtime: Runtime):
    ui.add_route(Route('/world', lambda request, **_: Response(content=runtime.world.json(), media_type='text/json')))
    ui.add_route(Route('/export', lambda request, **_: JSONResponse(content=runtime.persistence.dump())))

    def get_image(request: Request, **_):
        try:
            cam_id = request.path_params['id']
            camera = runtime.world.cameras.get(cam_id)
            if camera is None:
                return not_found
            for image in reversed(camera.images):
                if str(image.time) == request.path_params['timestamp']:
                    headers = {'cache-control': 'max-age=7776000'}  # 90 days
                    array = np.frombuffer(image.data, dtype=np.uint8)
                    img = cv2.imdecode(array, cv2.IMREAD_COLOR)[::2, ::2]
                    jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1].tostring()
                    return Response(content=jpeg, headers=headers, media_type='image/jpeg')
            return not_found
        except:
            log.exception('could not get image')
            raise

    ui.add_route(Route('/camera/{id}/{timestamp}', get_image))
