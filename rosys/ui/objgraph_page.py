from nicegui.elements.log import Log
from nicegui.ui import Ui
import rosys
import rosys.ui
from rosys import World
from rosys import event
from datetime import datetime
import objgraph
import random
import base64


class ObjgraphPage:
    ui: Ui = None  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        with self.ui.page('/objgraph'):
            self.ui.label('Analyze Memory')
            self.ui.button('log most common types', on_click=lambda: objgraph.show_most_common_types(limit=20))
            self.ui.button('log growth', on_click=lambda: objgraph.show_growth(limit=20))

            self.ui.input('Search Object', on_change=self.update_graph)
            self.graph = self.ui.row().style('width:40%')
            self.graph.visible = False

    def update_graph(self, event):
        objgraph.show_chain(objgraph.find_backref_chain(
            random.choice(objgraph.by_type(event.value)),  # just pick one of the found objects
            objgraph.is_proper_module),
            filename='/tmp/rosys_objgraph.png')
        with open('/tmp/rosys_objgraph.png', "rb") as f:
            image = base64.b64encode(f.read()).decode('utf-8')
        self.graph.clear()
        self.graph.visible = True
        with self.graph:
            self.ui.image('data:image/png;base64,'+str(image))
