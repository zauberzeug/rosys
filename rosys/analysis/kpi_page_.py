from itertools import groupby

import humanize
from matplotlib.colors import hsv_to_rgb, to_hex
from nicegui import ui

from .kpi_buckets import Month, TimeBucket, Week
from .kpi_logger import KpiLogger, str_to_date


class KpiPage:

    def __init__(self,
                 kpi_logger: KpiLogger,
                 *,
                 timespans: dict[int, str],
                 positives: dict[str, str],
                 negatives: dict[str, str],
                 title: str = '',
                 positive_title: str = '',
                 negative_title: str = '') -> None:
        self.kpi_logger = kpi_logger
        self.positives = positives
        self.negatives = negatives

        humanize.activate('de')

        @ui.page('/kpis')
        def page():
            with ui.row().style('margin:1em'):
                ui.markdown(f'### {title}').style('margin:1.5em;margin-top:-1.2em;')
                toggle = ui.toggle(timespans, value=list(timespans.keys())[0], on_change=lambda e: self.show(e.value))
            with ui.row():
                self.positive_chart = ui.chart({
                    'title': {'text': positive_title},
                    'chart': {'type': 'column'},
                    'yAxis': {'title': False},
                    'credits': False,
                    'exporting': False,
                })
                self.negative_chart = ui.chart({
                    'title': {'text': negative_title},
                    'chart': {'type': 'column'},
                    'plotOptions': {'series': {'stacking': 'normal'}},
                    'yAxis': {'title': False},
                    'colors': [to_hex(hsv_to_rgb((0, 1, i / 100.0))) for i in reversed(range(20, 120, 20))],
                    'credits': False,
                    'exporting': False,
                })
            ui.timer(5, lambda: self.show(toggle.value))

    def show(self, num_days: int) -> None:
        if num_days <= 7:
            time_buckets = self.kpi_logger.days[-num_days:]
        elif num_days % 7 == 0:
            weeks = [list(v) for _, v in groupby(self.kpi_logger.days,
                                                 key=lambda d: str_to_date(d.date).strftime('%y-%W'))][-num_days//7:]
            time_buckets = [Week.from_buckets(days_in_week) for days_in_week in weeks]
        elif num_days % 30 == 0 and num_days <= 90:
            months = [list(v) for _, v in groupby(self.kpi_logger.days,
                                                  key=lambda d: str_to_date(d.date).strftime('%y-%m'))][-num_days//30:]
            time_buckets = [Month.from_buckets(days_in_month) for days_in_month in months]
        else:
            raise ValueError(f'Unsupported number of days: {num_days}')

        keys = set(key for day in time_buckets for key in day.incidents if key in self.positives)
        data = {self.positives[key]: [day.incidents.get(key, 0) for day in time_buckets] for key in keys}
        self.positive_chart.options.xAxis.categories = [self._label(b) for b in time_buckets]
        self.positive_chart.options.series = [{'name': k, 'data': v} for k, v in sorted(data.items())]
        self.positive_chart.update()

        keys = set(key for day in time_buckets for key in day.incidents if key in self.negatives)
        data = {self.negatives[key]: [day.incidents.get(key, 0) for day in time_buckets] for key in keys}
        self.negative_chart.options.xAxis.categories = [self._label(b) for b in time_buckets]
        self.negative_chart.options.series = [{'name': k, 'data': v} for k, v in sorted(data.items())]
        self.negative_chart.update()

    @staticmethod
    def _label(time_bucket: TimeBucket) -> str:
        if isinstance(time_bucket, Month):
            return str_to_date(time_bucket.date).strftime('%b')
        elif isinstance(time_bucket, Week):
            return f'#{str_to_date(time_bucket.date).isocalendar()[1]}'
        else:
            return humanize.naturaldate(str_to_date(time_bucket.date))
