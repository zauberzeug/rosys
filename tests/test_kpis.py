from datetime import timedelta

import pytest
from rosys.analysis import Day, KpiLogger, date_to_str, str_to_date
from rosys.test import forward


async def test_log_incident(kpi_logger: KpiLogger):
    kpi_logger.increment('problem #42')
    assert len(kpi_logger.days) == 1
    assert kpi_logger.days[0].incidents['problem #42'] == 1

    await forward(seconds=1)
    kpi_logger.increment('problem #42')
    assert kpi_logger.days[0].incidents['problem #42'] == 2


def test_increment_on_rising_edge(kpi_logger: KpiLogger):
    day = kpi_logger.today()
    kpi_logger.increment_on_rising_edge('test_id', False)
    assert 'test_id' not in day.incidents
    kpi_logger.increment_on_rising_edge('test_id', True)
    assert day.incidents['test_id'] == 1
    kpi_logger.increment_on_rising_edge('test_id', True)
    assert day.incidents['test_id'] == 1, 'repetitive tracking "True" should not be counted'
    kpi_logger.increment_on_rising_edge('test_id', False)
    assert day.incidents['test_id'] == 1
    kpi_logger.increment_on_rising_edge('test_id', True)
    assert day.incidents['test_id'] == 2, 'if tracking "False" before, the state should be counted as "new"'
    kpi_logger.increment_on_rising_edge('test_id2', True)
    assert day.incidents['test_id'] == 2, 'each id should be tracked individually'
    assert day.incidents['test_id2'] == 1, 'the second id should be tracked'


@pytest.mark.parametrize('days, expected_days, expected_months, expected_sums', [
    (28, 28, 0, []),
    (90, 90, 0, []),
    (120, 103, 1, [17]),
    (180, 103, 3, [16, 31, 30]),
])
def test_packing_old_days_into_month(
        kpi_logger: KpiLogger, days: int, expected_days: int, expected_months: int, expected_sums: int):
    today = str_to_date('2022-03-13').date()
    kpi_logger.days = [
        Day(date=date_to_str(today - timedelta(days=d)), incidents=dict(problem=1, success=1))
        for d in range(days)
    ][::-1]
    kpi_logger.pack()
    assert len(kpi_logger.days) == expected_days
    assert len(kpi_logger.months) == expected_months
    for i, expected in enumerate(expected_sums):
        assert kpi_logger.months[i].incidents['problem'] == expected
        assert kpi_logger.months[i].incidents['success'] == expected
