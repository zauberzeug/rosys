from nicegui.ui import Ui
import logging
import html
import numpy as np
from ..actors import AsyncioMonitor
from .. import run

log = logging.getLogger('rosys.asyncio')


def prepare_data(timings):
    def calc_box(data: list):
        return {
            'low': min(data),
            'q1': np.percentile(data, 25),
            'median': np.percentile(data, 50),
            'q3': np.percentile(data, 75),
            'high': max(data),
        }
    timings = dict(sorted(timings.items(), key=lambda i: len(i[1]), reverse=True))
    names = [f'{html.escape(n)} ({len(w)} warnings):<br>{html.escape(w[0].details)}' for n, w in timings.items()]
    return names, [calc_box([w.duration * 1000 for w in t]) for t in timings.values()]


class AsyncioPage:
    ui: Ui = None
    asyncio_monitor: AsyncioMonitor

    def __init__(self) -> None:
        with self.ui.page('/asyncio'):
            with self.ui.row():
                self.ui.button('load', on_click=self.update)
                self.info_label = self.ui.label()
            self.chart = self.ui.chart(options={
                'title': False,
                'chart': {'type': 'boxplot'},
                'xAxis': {'categories': []},
                'yAxis': {'title': {'text': 'ms on UI thread'}},
                'series': {'data': []},
                'navigation': {'buttonOptions': {'enabled': False}},
                'legend': False,
                'credits': False,
            }).classes('fit').style('height:400px')

    async def update(self):
        names, data = await run.cpu_bound(prepare_data, self.asyncio_monitor.timings)
        self.chart.options['xAxis']['categories'][:] = names
        self.chart.options['series']['data'][:] = data
        await self.chart.view.update()
