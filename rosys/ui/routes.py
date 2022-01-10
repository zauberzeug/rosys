from nicegui.ui import Ui
import starlette
from rosys import Runtime

not_found = starlette.responses.Response(content='Not Found', status_code=404)

def setup(ui: Ui, runtime: Runtime):
    ui.add_route(starlette.routing.Route(
        '/world', lambda request, **_:
        starlette.responses.Response(content=runtime.world.json(exclude={'image_data'}), media_type='text/json')))
    ui.add_route(starlette.routing.Route(
        '/export', lambda request, **_:
        starlette.responses.JSONResponse(content=runtime.persistence.dump())))

    def get_frame(request, **_):
        cam_id = request.path_params['id']
        if cam_id not in runtime.world.cameras:
            return not_found
        for frame in reversed(runtime.world.cameras[cam_id].frames):
            if str(frame.time) == request.path_params['timestamp']:
                return starlette.responses.Response(content=frame.data, media_type='image/jpeg')
        return not_found

    ui.add_route(starlette.routing.Route('/camera/{id}/{timestamp}', get_frame))
