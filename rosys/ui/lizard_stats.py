from nicegui.ui import Ui
from nicegui.elements.chart import Chart
import psutil
from rosys.actors import Lizard


class LizardStats(Chart):
    ui: Ui
    lizard: Lizard

    def __init__(self) -> None:
        options = {
            'title': {'text': 'Lizard Statistics'},
            'chart': {'type': 'line'},
            'xAxis': {'labels': {'enabled': False}},
            'yAxis': {'categories': ['responsiveness'], 'title': {'text': 'ms'}},
            'series': [{'name': 'responsiveness', 'data': []}],
            'plotOptions': {
                'series': {'marker': {'enabled': False}}
            }
        }
        super().__init__(options)
        self.ui.timer(0.1, self.update)

    def update(self):
        self.view.options.series[0].data[:] = self.lizard.responsiveness_stats
