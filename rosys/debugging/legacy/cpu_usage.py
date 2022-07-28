import psutil
from nicegui import ui
from nicegui.elements.chart import Chart
from rosys import config


class CpuUsage(Chart):

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
        ui.timer(config.ui_update_interval, self.update)

    def update(self) -> None:
        for i, v in enumerate(psutil.cpu_percent(percpu=True)):
            self.options.series[i].data.append(v)
        [s.data.pop(0) for s in self.options.series if len(s.data) > 20]
