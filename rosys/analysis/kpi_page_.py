from abc import ABC, abstractmethod
from itertools import groupby
from typing import Sequence

import humanize
from matplotlib.colors import hsv_to_rgb, to_hex
from nicegui import ui

from .kpi_buckets import Month, TimeBucket, Week
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
                toggle = ui.toggle(self.timespans, value=list(self.timespans)[0], on_change=lambda e: show(e.value))
            with ui.row().classes('w-full'):
                positive_chart = ui.echart({
                    'title': {'text': self.positive_title},
                    'xAxis': {'type': 'category'},
                    'yAxis': {'type': 'value'},
                    'series': [],
                    'legend': {'bottom': 10},
                    'tooltip': {},
                })
                negative_chart = ui.echart({
                    'title': {'text': self.negative_title},
                    'xAxis': {'type': 'category'},
                    'yAxis': {'type': 'value'},
                    'series': [],
                    'color': [to_hex(hsv_to_rgb((0, 1, i / 100.0))) for i in reversed(range(20, 120, 20))],
                    'legend': {'bottom': 10},
                    'tooltip': {},
                })
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

                keys = set(key for day in time_buckets for key in day.incidents if key in self.positives)
                data = {self.positives[key]: [day.incidents.get(key, 0) for day in time_buckets] for key in keys}
                positive_chart.options['xAxis']['data'] = [_label(b) for b in time_buckets]
                positive_chart.options['series'] = [{'type': 'bar', 'name': k, 'data': v}
                                                    for k, v in sorted(data.items())]
                positive_chart.update()

                keys = set(key for day in time_buckets for key in day.incidents if key in self.negatives)
                data = {self.negatives[key]: [day.incidents.get(key, 0) for day in time_buckets] for key in keys}
                negative_chart.options['xAxis']['data'] = [_label(b) for b in time_buckets]
                negative_chart.options['series'] = [{'type': 'bar', 'stack': 'total', 'emphasis': {'focus': 'series'}, 'name': k, 'data': v}
                                                    for k, v in sorted(data.items())]
                negative_chart.update()

    @property
    def language(self) -> str:
        return 'en'

    @property
    @abstractmethod
    def title(self) -> str:
        pass

    @property
    @abstractmethod
    def positive_title(self) -> str:
        pass

    @property
    @abstractmethod
    def negative_title(self) -> str:
        pass

    @property
    @abstractmethod
    def positives(self) -> dict[str, str]:
        pass

    @property
    @abstractmethod
    def negatives(self) -> dict[str, str]:
        pass

    @property
    @abstractmethod
    def timespans(self) -> dict[int, str]:
        pass


def _label(time_bucket: TimeBucket) -> str:
    if isinstance(time_bucket, Month):
        return str_to_date(time_bucket.date).strftime(r'%b')
    if isinstance(time_bucket, Week):
        return f'#{str_to_date(time_bucket.date).isocalendar()[1]}'
    return humanize.naturaldate(str_to_date(time_bucket.date))
