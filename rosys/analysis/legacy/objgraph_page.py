import asyncio
import gc
from collections import Counter
from pathlib import Path

import objgraph
from nicegui import ui

from ... import run

SVG_FILE = Path('/tmp/rosys_objgraph.svg')


def objgraph_page() -> None:

    @ui.page('/objgraph')
    def page() -> None:
        async def update_graph() -> None:
            class_search_result.content = await create_graph(class_search.value)

        async def refresh_stats() -> None:
            loop = asyncio.get_running_loop()
            growth, leaking, overall, counts = await loop.run_in_executor(None, get_objgraph_stats)
            await update_graph()
            growth_label.set_text(growth)
            leaking_label.set_text(leaking)
            overall_label.set_text(overall)
            if counts is None:
                counts_markdown.content = ''
                most_common_search_result.content = ''
            else:
                highest = [f'{c[1]}: {c[0]}' for c in counts.most_common()[:5]]
                counts_markdown.set_content('new obj occurrence: ' + '\n'.join(highest))
                most_common_search_result.content = SVG_FILE.read_text()

        t = ui.timer(10, refresh_stats, active=False)
        ui.switch('track objects (every 10 s)').bind_value_to(t, 'active')
        with ui.column():
            growth_label = ui.label()
            leaking_label = ui.label()
            overall_label = ui.label()
            counts_markdown = ui.markdown()

        class_search = ui.input('Search Class', value='Client')
        ui.button('Update Graph', on_click=update_graph)
        class_search_result = ui.html()
        most_common_search_result = ui.html()


@run.awaitable
def create_graph(search_term: str) -> str:
    gc.collect()
    objects = objgraph.by_type(search_term)
    content = ''
    for obj in objects:
        chain = objgraph.find_backref_chain(obj, objgraph.is_proper_module)
        objgraph.show_chain(chain, filename=str(SVG_FILE))
        content += SVG_FILE.read_text()
    return content


def get_objgraph_stats() -> tuple[str, str, str, Counter[str] | None]:
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
