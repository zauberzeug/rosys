from nicegui.ui import Ui
from nicegui.elements.chart import Chart
from rosys.actors import Lizard


class LizardStats(Chart):
    ui: Ui
    lizard: Lizard

    def __init__(self) -> None:
        options = {
            'title': {'text': 'Lizard Statistics'},
            'chart': {'type': 'line', 'animation': False},
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
        self.view.options.series[0].data.append(max(self.lizard.responsiveness_stats))
        if len(self.view.options.series[0].data) > 20:
            self.view.options.series[0].data.pop(0)
