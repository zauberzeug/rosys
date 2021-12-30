from nicegui.ui import Ui
import starlette
from rosys import Runtime


def setup(ui: Ui, runtime: Runtime):
    ui.add_route(starlette.routing.Route(
        '/world', lambda request, **_:
        starlette.responses.Response(content=runtime.world.json(exclude={'image_data'}), media_type='text/json')))
    ui.add_route(starlette.routing.Route(
        '/export', lambda request, **_:
        starlette.responses.JSONResponse(content=runtime.persistence.dump())))
