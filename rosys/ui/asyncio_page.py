from nicegui.ui import Ui
from ..actors import AsyncioMonitor
import logging
import numpy as np

log = logging.getLogger('rosys.asyncio')


class AsyncioPage:
    ui: Ui = None
    asyncio_monitor: AsyncioMonitor

    def __init__(self) -> None:
        with self.ui.page('/asyncio'):
            self.ui.timer(2, self.refresh_stats)
            self.chart = self.ui.chart(options={
                'chart': {'type': 'boxplot'},
                'xAxis': {'categories': [], },
                'yAxis': {'title': {'text': 'ms on UI thread'}, },
                'series': {'data': []},
            }).classes('fit')

    async def refresh_stats(self):
        timings = dict(sorted(self.asyncio_monitor.timings.items(), key=lambda i: len(i[1]), reverse=True))
        names = [f'{n} ({len(w)} warnings)' for n, w in timings.items()]
        self.chart.options['xAxis']['categories'][:] = names
        data = [self.calc_box([w.millis for w in t]) for t in timings.values()]
        self.chart.options['series']['data'][:] = data

    def calc_box(self, data: list):
        return {
            'low': min(data),
            'q1': np.percentile(data, 25),
            'median': np.percentile(data, 50),
            'q3': np.percentile(data, 75),
            'high': max(data),
        }
