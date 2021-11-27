from nicegui.ui import Ui
from pyloot import PyLoot
from pyloot.server import PyLootServer
from starlette.applications import Starlette
from starlette.middleware.wsgi import WSGIMiddleware
from starlette.routing import Mount


class PylootPage():
    ui: Ui = None  # will be set by rosys.ui.configure
    app: Starlette

    def __init__(self) -> None:
        self.pyloot = PyLoot(server=PyLootServer(disable_response_gzip=True))  # gzip is done by starlette
        self.ui.on_startup(self.pyloot.start)
        self.app.routes.insert(0, Mount("/pyloot", app=WSGIMiddleware(self.pyloot_wrapper)))

    def pyloot_wrapper(self, wsgi_environ, start_response):
        pyloot_environ = wsgi_environ.copy()
        pyloot_environ["wsgi.multiprocess"] = False
        wsgi = self.pyloot.get_wsgi()
        return wsgi(pyloot_environ, start_response)
