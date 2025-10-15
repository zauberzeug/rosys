from __future__ import annotations

import logging
import math
from abc import ABC
from dataclasses import dataclass

from nicegui import Event, ui

from ... import rosys
from ...geometry import GeoPoint, GeoPose
from .nmea import GpsQuality

SECONDS_DAY = 86400
SECONDS_HALF_DAY = 43200


@dataclass(slots=True, kw_only=True)
class GnssMeasurement:
    time: float
    gnss_time: float
    pose: GeoPose
    latitude_std_dev: float = 0.0
    longitude_std_dev: float = 0.0
    heading_std_dev: float = 0.0
    gps_quality: GpsQuality = GpsQuality.INVALID
    num_satellites: int = 0
    hdop: float = 0.0
    altitude: float = 0.0

    @property
    def point(self) -> GeoPoint:
        return self.pose.point

    @property
    def heading(self) -> float:
        return self.pose.heading

    @property
    def age(self) -> float:
        return ((self.gnss_time - rosys.time() + SECONDS_HALF_DAY) % SECONDS_DAY) - SECONDS_HALF_DAY


class Gnss(ABC):
    """A GNSS module that provides measurements from a GNSS receiver."""

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.gnss')
        self.last_measurement: GnssMeasurement | None = None

        self.NEW_MEASUREMENT = Event[GnssMeasurement]()
        """a new measurement has been received (argument: ``GnssMeasurement``)"""

    @property
    def is_connected(self) -> bool:
        return False

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('GNSS').classes('text-center text-bold')
            with ui.grid(columns='auto auto').classes('gap-y-1 w-2/5'):
                ui.label('Connected:')
                ui.icon('close').bind_name_from(self, 'is_connected',
                                                lambda x: 'check' if x else 'close')
                ui.label('Position:')
                with ui.column().classes('gap-y-0'):
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'{math.degrees(x.pose.lat):.8f}˚' if x else '-')
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'± {x.latitude_std_dev:.3f}m' if x else '-')
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'{math.degrees(x.pose.lon):.8f}˚' if x else '-')
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'± {x.longitude_std_dev:.3f}m' if x else '-')
                ui.label('Heading:')
                with ui.column().classes('gap-y-0'):
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'{math.degrees(x.pose.heading):.2f}˚' if x else '-')
                    ui.label().bind_text_from(self, 'last_measurement',
                                              lambda x: f'± {x.heading_std_dev:.2f}˚' if x else '-')
                ui.label('Quality:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: x.gps_quality.name if x else '')
                ui.label('Satellites:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: str(x.num_satellites) if x else '-')
                ui.label('HDOP:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{x.hdop:.2f}' if x else '-')
                ui.label('Altitude:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{x.altitude:.3f}m' if x else '-')
                ui.label('Last update:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{rosys.time() - x.time:.2f}s' if x else '')
