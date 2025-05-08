from __future__ import annotations

from datetime import date, datetime, timezone
from itertools import groupby
from typing import Any

from rosys import persistence

from .. import rosys
from .kpi_buckets import Day, Month


def date_to_str(date_: date) -> str:
    return date_.isoformat()


def str_to_date(date_: str) -> datetime:
    return datetime.fromisoformat(date_)


class KpiLogger(persistence.Persistable):

    def __init__(self) -> None:
        super().__init__()

        self.states: dict[str, bool] = {}
        self.days: list[Day] = []
        self.months: list[Month] = []

    def backup_to_dict(self) -> dict:
        self.pack()
        return {
            'days': persistence.to_dict(self.days),
            'months': persistence.to_dict(self.months),
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        persistence.replace_list(self.days, Day, data.get('days', []))
        persistence.replace_list(self.months, Month, data.get('months', []))

    def pack(self) -> None:
        """pack old days into months (as long as there are at least 3 unpacked months)"""
        while True:
            month_groups = [list(days) for _, days in groupby(self.days, key=lambda d: d.date[:-3])]
            if len(month_groups) <= 4:
                break
            month = Month.from_buckets(month_groups[0])
            self.months.append(month)
            for day in month_groups[0]:
                self.days.remove(day)
            self.request_backup()

    def today(self) -> Day:
        date_ = date_to_str(datetime.fromtimestamp(rosys.time(), tz=timezone.utc).date())
        if not self.days or self.days[-1].date != date_:
            self.days.append(Day(date=date_))
        return self.days[-1]

    def increment(self, key: str) -> None:
        day = self.today()
        day.incidents[key] = day.incidents.get(key, 0) + 1
        self.request_backup()

    def increment_on_rising_edge(self, key: str, new_state: bool) -> None:
        if new_state and not self.states.get(key, False):
            self.increment(key)
        self.states[key] = new_state
