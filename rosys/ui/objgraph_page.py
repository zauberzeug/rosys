from nicegui.ui import Ui
from nicegui.elements.image import Image
import objgraph
import random
import base64
import asyncio
import logging
import gc
from collections import Counter

log = logging.getLogger('rosys.objgraph_page')


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
                self.counts = self.ui.markdown()

            self.class_search = self.ui.input('Search Class', on_change=self.update_graph)
            self.class_search_result = self.ui.image().props('contain').style('height:400px;width:100%')
            self.class_search_result.visible = False
            self.most_common_search_result = self.ui.image().props('contain').style('height:400px;width:100%')
            self.most_common_search_result.visible = False

    def update_graph(self, _=None):
        if not self.class_search.value:
            return
        objgraph.show_chain(objgraph.find_backref_chain(
            random.choice(objgraph.by_type(self.class_search.value)),  # just pick one of the found objects
            objgraph.is_proper_module),
            filename='/tmp/rosys_objgraph_class_search.png')
        self.load_image('/tmp/rosys_objgraph_class_search.png', self.class_search_result)

    def load_image(self, file: str, img: Image):
        try:
            with open(file, 'rb') as f:
                data = base64.b64encode(f.read()).decode('utf-8')
            img.clear()
            img.visible = True
            img.set_source('data:image/png;base64,'+str(data))
        except:
            log.exception('could not load image ' + file)

    def get_objgraph_stats(self):
        gc.collect()
        growth_objs = objgraph.growth(4)
        growth = 'object growth every 10 sec: '
        for obj in growth_objs:
            growth += f'{obj[0]}: +{obj[2]}  '
        leaking = 'obj without refs: '
        for o, c in list(objgraph.typestats(objgraph.get_leaking_objects()).items())[:4]:
            leaking += f'{o}: {c}  '
        overall = 'overall obj count: '
        for o, c in list(objgraph.typestats().items())[:4]:
            overall += f'{o}: {c}  '
        if growth_objs:
            biggest_growth = growth_objs[0][0]
            new = objgraph.at_addrs(objgraph.get_new_ids()[biggest_growth])
            counts = Counter(str(i) for i in new)
            if counts:
                most_common = counts.most_common()[0][0]
                unique = {str(i): i for i in new}
                # build chain for most common object
                objgraph.show_chain(objgraph.find_backref_chain(
                    unique[most_common],
                    objgraph.is_proper_module),
                    filename='/tmp/rosys_objgraph_most_common.png'
                )

        return growth, leaking, overall, counts

    async def refresh_stats(self):
        loop = asyncio.get_running_loop()
        growth, leaking, overall, counts = await loop.run_in_executor(None, self.get_objgraph_stats)
        await loop.run_in_executor(None, self.update_graph)
        self.growth.set_text(growth)
        self.leaking.set_text(leaking)
        self.overall.set_text(overall)
        highest = [f'{c[1]}: {c[0]}' for c in counts.most_common()[:5]]
        self.counts.set_content('new obj occurrence: ' + '\n'.join(highest))
        if counts:
            self.load_image('/tmp/rosys_objgraph_most_common.png', self.most_common_search_result)
        else:
            self.most_common_search_result.visible = False
