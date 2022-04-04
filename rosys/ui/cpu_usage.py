import psutil
from nicegui.elements.chart import Chart
from nicegui.ui import Ui

from .settings import update_interval


class CpuUsage(Chart):
    ui: Ui  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        super().__init__({
            'title': {'text': 'CPU', 'floating': True, 'y': 20},
            'chart': {'type': 'line', 'animation': False},
            'xAxis': {'labels': False},
            'yAxis': {'min': 0, 'max': 100, 'title': {'text': '%'}},
            'series': [{'data': [p]} for p in psutil.cpu_percent(percpu=True)],
            'plotOptions': {'series': {'marker': False}},
            'navigation': {'buttonOptions': {'enabled': False}},
            'legend': False,
            'credits': False,
        })
        self.ui.timer(update_interval, self.update)

    def update(self) -> bool:
        for i, v in enumerate(psutil.cpu_percent(percpu=True)):
            self.options.series[i].data.append(v)
        [s.data.pop(0) for s in self.options.series if len(s.data) > 20]
        return False  # NOTE: avoid JustPy page_update
