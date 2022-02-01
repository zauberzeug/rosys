from nicegui.ui import Ui
import starlette
from rosys import Runtime
import logging

log = logging.getLogger('rosys.routes')
not_found = starlette.responses.Response(content='Not Found', status_code=404)


def setup(ui: Ui, runtime: Runtime):
    ui.add_route(starlette.routing.Route(
        '/world', lambda request, **_:
        starlette.responses.Response(content=runtime.world.json(), media_type='text/json')))
    ui.add_route(starlette.routing.Route(
        '/export', lambda request, **_:
        starlette.responses.JSONResponse(content=runtime.persistence.dump())))

    def get_image(request, **_):
        try:
            cam_id = request.path_params['id']
            if cam_id not in runtime.world.usb_cameras:
                return not_found
            for image in reversed(runtime.world.usb_cameras[cam_id].images):
                if str(image.time) == request.path_params['timestamp']:
                    return starlette.responses.Response(content=image.data, media_type='image/jpeg')
            return not_found
        except:
            log.exception('could not get image')
            raise

    ui.add_route(starlette.routing.Route('/camera/{id}/{timestamp}', get_image))
