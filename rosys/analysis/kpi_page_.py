from abc import ABC, abstractmethod
from collections.abc import Sequence
from itertools import groupby

import humanize
from matplotlib import colormaps
from matplotlib.colors import to_hex
from nicegui import ui

from .kpi_buckets import Month, TimeBucket, Week
from .kpi_chart import KpiChart
from .kpi_logger import KpiLogger, str_to_date


class kpi_page(ABC):

    def __init__(self, kpi_logger: KpiLogger) -> None:
        @ui.page('/kpis')
        def page():
            if self.language == 'de':
                humanize.activate('de')
            else:
                humanize.deactivate()
            with ui.row().style('margin:1em'):
                ui.markdown(f'### {self.title}').style('margin:1.5em;margin-top:-1.2em;')
                toggle = ui.toggle(self.timespans, value=next(iter(self.timespans)), on_change=lambda e: show(e.value))
            with ui.row().classes('w-full'):
                ui_charts = []
                for chart in self.charts:
                    ui_charts.append(ui.echart({
                        'title': {'text': chart.title},
                        'xAxis': {'type': 'category'},
                        'yAxis': {'type': 'value', 'name': chart.unit},
                        'legend': {'bottom': 10},
                        'tooltip': {},
                        'color': [to_hex(colormaps[chart.colormap or 'viridis'](i / 10)) for i in range(9, -1, -2)],
                        'series': [],
                    }))
            ui.timer(5, lambda: show(toggle.value))

            def show(num_days: int) -> None:
                time_buckets: Sequence[TimeBucket]
                if num_days <= 7:
                    time_buckets = kpi_logger.days[-num_days:]
                elif num_days % 7 == 0:
                    weeks = [list(v) for _, v in groupby(kpi_logger.days,
                                                         key=lambda d: str_to_date(d.date).strftime('%y-%W'))][-num_days//7:]
                    time_buckets = [Week.from_buckets(days_in_week) for days_in_week in weeks]
                elif num_days % 30 == 0 and num_days <= 90:
                    months = [list(v) for _, v in groupby(kpi_logger.days,
                                                          key=lambda d: str_to_date(d.date).strftime('%y-%m'))][-num_days//30:]
                    time_buckets = [Month.from_buckets(days_in_month) for days_in_month in months]
                else:
                    raise ValueError(f'Unsupported number of days: {num_days}')

                for chart, ui_chart in zip(self.charts, ui_charts, strict=True):
                    keys = set(key for day in time_buckets for key in day.incidents if key in chart.indicators)
                    data = {chart.indicators[key]: [day.incidents.get(key, 0) for day in time_buckets] for key in keys}
                    styling = {'type': 'bar', 'stack': 'total', 'emphasis': {'focus': 'series'}}
                    ui_chart.options['xAxis']['data'] = [_label(b) for b in time_buckets]
                    ui_chart.options['series'] = [
                        {**styling, 'name': k, 'data': [round(item * chart.scale, 2) for item in v]}
                        for k, v in sorted(data.items())
                    ]
                    ui_chart.update()

    @property
    def language(self) -> str:
        return 'en'

    @property
    @abstractmethod
    def title(self) -> str:
        pass

    @property
    @abstractmethod
    def timespans(self) -> dict[int, str]:
        pass

    @property
    @abstractmethod
    def charts(self) -> list[KpiChart]:
        pass


def _label(time_bucket: TimeBucket) -> str:
    if isinstance(time_bucket, Month):
        return str_to_date(time_bucket.date).strftime(r'%b')
    if isinstance(time_bucket, Week):
        return f'#{str_to_date(time_bucket.date).isocalendar()[1]}'
    return humanize.naturaldate(str_to_date(time_bucket.date))
