from nicegui.ui import Ui
from nicegui.elements.chart import Chart
import psutil


class CpuUsage(Chart):
    ui: Ui  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        cpus = len(psutil.cpu_percent(percpu=True))
        options = {
            'title': False,
            'chart': {'type': 'line'},
            'xAxis': {'labels': {'enabled': False}},
            'yAxis': {'categories': [f'cpu_{i}' for i in range(cpus)], 'max': 100, 'min': 0, 'title': {'text': 'utilization'}},
            'series': [{'name': f'cpu_{i}', 'data': []} for i in range(cpus)],
            'plotOptions': {
                'series': {'marker': {'enabled': False}}
            }
        }
        super().__init__(options)
        self.ui.timer(0.1, self.update)

    def update(self):
        utilizations = psutil.cpu_percent(percpu=True)
        for i, v in enumerate(utilizations):
            self.view.options.series[i].data.append(v)
            if (len(self.view.options.series[i].data) > 20):
                self.view.options.series[i].data.pop(0)
