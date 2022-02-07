from nicegui.ui import Ui
from nicegui.elements.chart import Chart
from rosys.actors import Lizard


class LizardStats(Chart):
    ui: Ui
    lizard: Lizard

    def __init__(self) -> None:
        super().__init__({
            'title': {'text': 'Lizard', 'floating': True, 'y': 20},
            'chart': {'type': 'line', 'animation': False},
            'xAxis': {'labels': False},
            'yAxis': {'min': 0, 'title': {'text': 'ms'}},
            'series': [
                {'name': 'responsiveness', 'data': []},
                {'name': 'update', 'data': []},
                {'name': 'processing', 'data': []},
            ],
            'plotOptions': {'series': {'marker': False}},
            'navigation': {'buttonOptions': {'enabled': False}},
            'credits': False,
        })
        self.ui.timer(0.1, self.update)

    def update(self):
        self.options.series[0].data.append(max(self.lizard.responsiveness_stats or [0]) * 1000)
        self.options.series[1].data.append(max(self.lizard.update_stats or [0]) * 1000)
        self.options.series[2].data.append(max(self.lizard.processing_stats or [0]) * 1000)
        [s.data.pop(0) for s in self.options.series if len(s.data) > 20]
