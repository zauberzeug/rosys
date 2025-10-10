from __future__ import annotations

import math

import numpy as np
from nicegui import ui

from ... import rosys
from ...driving.driver import PoseProvider
from ...geometry import GeoPose
from .gnss import Gnss, GnssMeasurement
from .nmea import GpsQuality


class GnssSimulation(Gnss):
    """Simulation of a GNSS receiver."""

    def __init__(self, *,
                 wheels: PoseProvider,
                 lat_std_dev: float = 0.01,
                 lon_std_dev: float = 0.01,
                 heading_std_dev: float = 0.01,
                 interval: float = 1.0,
                 latency: float = 0.0,
                 gps_quality: GpsQuality = GpsQuality.RTK_FIXED) -> None:
        """
        :param wheels: the wheels to use for the simulation
        :param lat_std_dev: the standard deviation of the latitude in meters
        :param lon_std_dev: the standard deviation of the longitude in meters
        :param heading_std_dev: the standard deviation of the heading in degrees
        :param gps_quality: the quality of the GPS signal
        :param interval: the interval between measurements in seconds
        :param latency: the simulated measurement latency in seconds
        """
        super().__init__()
        self.wheels = wheels
        self._is_connected = True
        self._lat_std_dev = lat_std_dev
        self._lon_std_dev = lon_std_dev
        self._heading_std_dev = heading_std_dev
        self._gps_quality = gps_quality
        self._latency = latency
        self.last_measurement: GnssMeasurement | None = None
        rosys.on_repeat(self.simulate, interval)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    async def simulate(self) -> None:
        if not self.is_connected:
            return
        geo_pose = GeoPose.from_pose(self.wheels.pose)
        noise_lat = np.random.normal(0, self._lat_std_dev)
        noise_lon = np.random.normal(0, self._lon_std_dev)
        noise_magnitude = np.sqrt(noise_lat**2 + noise_lon**2)
        noise_direction = math.atan2(noise_lon, noise_lat)
        noise_point = geo_pose.point.polar(noise_magnitude, noise_direction)
        noise_heading = np.random.normal(geo_pose.heading, math.radians(self._heading_std_dev))
        noise_pose = GeoPose(lat=noise_point.lat, lon=noise_point.lon, heading=noise_heading)
        timestamp = rosys.time()
        self.last_measurement = GnssMeasurement(
            time=timestamp,
            gnss_time=timestamp,
            pose=noise_pose,
            latitude_std_dev=self._lat_std_dev,
            longitude_std_dev=self._lon_std_dev,
            heading_std_dev=self._heading_std_dev,
            gps_quality=self._gps_quality,
        )
        if self._latency:
            await rosys.sleep(self._latency)
        self.NEW_MEASUREMENT.emit(self.last_measurement)

    def developer_ui(self) -> None:
        super().developer_ui()
        with ui.column():
            ui.label('Simulation').classes('text-center text-bold')
            with ui.column().classes('gap-y-1'):
                ui.checkbox('Connected').bind_value(self, '_is_connected')
                ui.number(label='Latitude Std Dev', format='%.3f', prefix='± ', suffix='m') \
                    .bind_value(self, '_lat_std_dev').classes('w-4/5')
                ui.number(label='Longitude Std Dev', format='%.3f', prefix='± ', suffix='m') \
                    .bind_value(self, '_lon_std_dev').classes('w-4/5')
                ui.number(label='Heading Std Dev', format='%.2f', prefix='± ', suffix='°') \
                    .bind_value(self, '_heading_std_dev').classes('w-4/5')
                ui.select({quality: quality.name for quality in GpsQuality}, label='Quality') \
                    .bind_value(self, '_gps_quality').classes('w-4/5')
                ui.number(label='Latency', format='%.2f', suffix='s') \
                    .bind_value(self, '_latency').classes('w-4/5')
