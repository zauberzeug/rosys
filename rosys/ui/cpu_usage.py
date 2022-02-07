from nicegui.ui import Ui
from nicegui.elements.chart import Chart
import psutil


class CpuUsage(Chart):
    ui: Ui  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        cpus = len(psutil.cpu_percent(percpu=True))
        super().__init__({
            'title': False,
            'chart': {'type': 'line'},
            'xAxis': {'labels': False},
            'yAxis': {'categories': [f'cpu_{i}' for i in range(cpus)], 'max': 100, 'min': 0, 'title': {'text': 'utilization'}},
            'series': [{'name': f'cpu_{i}', 'data': []} for i in range(cpus)],
            'plotOptions': {
                'series': {'marker': False}
            }
        })
        self.ui.timer(0.1, self.update)

    def update(self):
        utilizations = psutil.cpu_percent(percpu=True)
        for i, v in enumerate(utilizations):
            self.options.series[i].data.append(v)
        [s.data.pop(0) for s in self.options.series if len(s.data) > 20]
