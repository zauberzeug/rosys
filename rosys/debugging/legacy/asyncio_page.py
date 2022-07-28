import html
import logging

import numpy as np
from nicegui import ui
from rosys import run

from .asyncio_monitor import AsyncioMonitor

log = logging.getLogger('rosys.asyncio_page')


def prepare_data(timings) -> tuple[list[str], list[dict[str, float]]]:
    def calc_box(data: list) -> dict[str, float]:
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


class AsyncioPage(ui.page):

    def __init__(self, asyncio_monitor: AsyncioMonitor) -> None:
        super().__init__('/asyncio')

        self.asyncio_monitor = asyncio_monitor

        with self:
            with ui.row():
                ui.button('load', on_click=self.update)
                self.info_label = ui.label()
            self.chart = ui.chart(options={
                'title': False,
                'chart': {'type': 'boxplot'},
                'xAxis': {'categories': []},
                'yAxis': {'title': {'text': 'ms on UI thread'}},
                'series': {'data': []},
                'navigation': {'buttonOptions': {'enabled': False}},
                'legend': False,
                'credits': False,
            }).classes('fit').style('height:400px')

    async def update(self) -> None:
        names, data = await run.cpu_bound(prepare_data, self.asyncio_monitor.timings)
        self.chart.options['xAxis']['categories'][:] = names
        self.chart.options['series']['data'][:] = data
        await self.chart.view.update()
