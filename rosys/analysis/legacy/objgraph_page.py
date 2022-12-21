import asyncio
import gc
import logging
import random
from collections import Counter
from pathlib import Path
from typing import Optional

import objgraph
from nicegui import ui

log = logging.getLogger('rosys.objgraph_page')

SVG_FILE = Path('/tmp/rosys_objgraph.svg')


class ObjgraphPage:

    def __init__(self) -> None:
        @ui.page('/objgraph')
        def objgraph_page() -> None:
            t = ui.timer(10, self.refresh_stats, active=False)
            ui.switch('track objects (every 10 s)').bind_value_to(t, 'active')
            with ui.column():
                self.growth = ui.label()
                self.leaking = ui.label()
                self.overall = ui.label()
                self.counts = ui.markdown()

            self.class_search = ui.input('Search Class')
            ui.button('Update Graph', on_click=self.update_graph)
            self.class_search_result = ui.html()
            self.most_common_search_result = ui.html()

    def update_graph(self) -> None:
        gc.collect()
        objects = objgraph.by_type(self.class_search.value)
        if not objects:
            return
        chain = objgraph.find_backref_chain(random.choice(objects), objgraph.is_proper_module)
        objgraph.show_chain(chain, filename=str(SVG_FILE))
        self.class_search_result.content = SVG_FILE.read_text()

    def get_objgraph_stats(self) -> tuple[str, str, str, Optional[Counter[str]]]:
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
                chain = objgraph.find_backref_chain(unique[most_common], objgraph.is_proper_module)
                objgraph.show_chain(chain, filename=str(SVG_FILE))
        else:
            counts = None

        return growth, leaking, overall, counts

    async def refresh_stats(self) -> None:
        loop = asyncio.get_running_loop()
        growth, leaking, overall, counts = await loop.run_in_executor(None, self.get_objgraph_stats)
        await loop.run_in_executor(None, self.update_graph)
        self.growth.set_text(growth)
        self.leaking.set_text(leaking)
        self.overall.set_text(overall)
        highest = [f'{c[1]}: {c[0]}' for c in counts.most_common()[:5]]
        self.counts.set_content('new obj occurrence: ' + '\n'.join(highest))
        self.most_common_search_result.content = SVG_FILE.read_text() if counts else ''
