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


class AsyncioPage(ui.element):

    def __init__(self, asyncio_monitor: AsyncioMonitor) -> None:
        super().__init__()

        async def update() -> None:
            result = await run.cpu_bound(prepare_data, asyncio_monitor.timings)
            assert result is not None
            chart.options['xAxis']['categories'][:] = result[0]
            chart.options['series']['data'][:] = result[1]
            chart.update()

        with self:
            ui.button('load', on_click=update)
            chart = ui.highchart(options={
                'title': False,
                'chart': {'type': 'boxplot'},
                'xAxis': {'categories': []},
                'yAxis': {'title': {'text': 'ms on UI thread'}},
                'series': {'data': []},
                'navigation': {'buttonOptions': {'enabled': False}},
                'legend': False,
                'credits': False,
            }).classes('fit').style('height:400px')
