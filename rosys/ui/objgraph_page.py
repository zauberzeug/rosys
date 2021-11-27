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
import asyncio


class ObjgraphPage:
    ui: Ui = None  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        with self.ui.page('/objgraph'):
            t = self.ui.timer(10, self.refresh_stats, active=False)
            self.ui.switch('track objects (every 10 s)').bind_value_to(t, 'active')
            with self.ui.column():
                self.growth = self.ui.label()
                self.leaking = self.ui.label()
                self.overall = self.ui.label()

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

    @staticmethod
    def get_objgraph_stats():
        growth = 'object growth every 10 sec: '
        for obj in objgraph.growth(4):
            growth += f'{obj[0]}: +{obj[2]}  '

        leaking = 'obj without refs: '
        for o, c in list(objgraph.typestats(objgraph.get_leaking_objects()).items())[:4]:
            leaking += f'{o}: {c}  '

        overall = 'overall obj count: '
        for o, c in list(objgraph.typestats().items())[:4]:
            overall += f'{o}: {c}  '

        return growth, leaking, overall

    async def refresh_stats(self):
        loop = asyncio.get_running_loop()
        growth, leaking, overall = await loop.run_in_executor(None, self.get_objgraph_stats)
        self.growth.set_text(growth)
        self.leaking.set_text(leaking)
        self.overall.set_text(overall)
