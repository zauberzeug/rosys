from datetime import datetime, timedelta
from random import randint

from nicegui import ui

from rosys.analysis import Day, KpiChart, KpiLogger, date_to_str
from rosys.analysis import kpi_page as rosys_kpi_page


def generate_example_kpis(logger: KpiLogger) -> None:
    logger.days = [
        Day(
            date=date_to_str(datetime.today().date() - timedelta(days=i)),
            incidents={
                'wheels_slipped': randint(0, 100),
                'wheels_blocking': randint(0, 15),
                'tests_run': randint(0, 30),
                'finished_task': randint(0, 5),
                'connection_established': randint(0, 200),
                'bumps': randint(0, 50),
            },
        ) for i in range(7 * 3)
    ][::-1]


class kpi_page(rosys_kpi_page):

    @property
    def title(self) -> str:
        return 'KPI Example Page'

    @property
    def charts(self) -> list[KpiChart]:
        positives = KpiChart(title='Positive Events', indicators={
            'finished_task': 'Finished Tasks',
        }, color='Greens')
        neutral_events = KpiChart(title='Neutral Events', indicators={
            'connection_established': 'Connection was established',
            'tests_run': 'Tests'
        })
        negatives = KpiChart(title='Exceptions', indicators={
            'wheels_slipped': 'Wheels Slipping',
            'wheels_blocking': 'Wheels Blocking',
            'bumps': 'Robot bumped into something',
        }, color='Reds')
        return [positives, neutral_events, negatives]

    @property
    def timespans(self) -> dict[int, str]:
        return {
            7: '7 days',
            14: '2 weeks',
            21: '3 weeks',
        }


kpi_logger = KpiLogger()
generate_example_kpis(kpi_logger)
kpi_page(kpi_logger)


ui.run(title='KPI Example', )
