from __future__ import annotations

import logging
import math
from abc import ABC
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

import serial
from nicegui import ui
from serial.tools import list_ports

from .. import rosys
from ..event import Event
from ..geometry import GeoPoint, GeoPose, Pose
from ..run import io_bound

if TYPE_CHECKING:
    from ..hardware import WheelsSimulation


class GpsQuality(IntEnum):
    INVALID = 0
    GPS = 1
    DGPS = 2
    PPS = 3
    RTK_FIXED = 4
    RTK_FLOAT = 5
    DEAD_RECKONING = 6
    MANUAL = 7
    SIMULATION = 8


@dataclass
class GnssMeasurement:
    time: float
    pose: GeoPose
    latitude_std_dev: float = 0.0
    longitude_std_dev: float = 0.0
    heading_std_dev: float = 0.0
    gps_qual: GpsQuality = GpsQuality.INVALID
    num_satellites: int = 0
    hdop: float = 0.0
    altitude: float = 0.0

    @property
    def point(self) -> GeoPoint:
        return self.pose.point

    @property
    def heading(self) -> float:
        return self.pose.heading


class Gnss(ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.gnss')
        self.last_measurement: GnssMeasurement | None = None

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received"""

    @property
    def is_connected(self) -> bool:
        return False

    def developer_ui(self) -> None:
        ui.label('GNSS').classes('text-center text-bold')
        with ui.grid(columns='auto auto').classes('gap-y-1 w-2/5'):
            ui.label('Connected:')
            ui.icon('close').bind_name_from(self, 'is_connected',
                                            lambda x: 'check' if x else 'close')
            ui.label('Position:')
            with ui.column().classes('gap-y-0'):
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{math.degrees(x.pose.lat):.6f}˚' if x else '-')
                ui.label().bind_text_from(self.last_measurement, 'latitude_std_dev',
                                          lambda x: f'± {x:.3f}m' if x else '-')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{math.degrees(x.pose.lon):.6f}˚' if x else '-')
                ui.label().bind_text_from(self.last_measurement, 'longitude_std_dev',
                                          lambda x: f'± {x:.3f}m' if x else '-')
            ui.label('Heading:')
            with ui.column().classes('gap-y-0'):
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda x: f'{math.degrees(x.pose.heading):.2f}˚' if x else '-')
                ui.label().bind_text_from(self.last_measurement, 'heading_std_dev',
                                          lambda x: f'± {x:.2f}˚' if x else '-')
            ui.label('Quality:')
            ui.label().bind_text_from(self.last_measurement, 'gps_qual',
                                      lambda x: x.name if x else '')
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


class GnssHardware(Gnss):
    """
    This hardware module connects to a Septentrio SimpleRTK3b (Mosaic-H) GNSS receiver.
    """

    def __init__(self, *, antenna_pose: Pose | None) -> None:
        """
        :param antenna_pose: the position of the main antenna in the robot's coordinate system (yaw = direction to auxiliary antenna)
        """
        super().__init__()
        self.antenna_pose = antenna_pose or Pose(x=0.0, y=0.0, yaw=0.0)
        self._antenna_distance = math.sqrt(self.antenna_pose.x**2 + self.antenna_pose.y**2)
        """the distance from the robot's center to the main antenna"""
        self._antenna_angle = math.pi + math.atan2(self.antenna_pose.y, self.antenna_pose.x) - self.antenna_pose.yaw
        """the angle from the robot's center to the main antenna"""
        serial_port = self._find_device_port()
        assert serial_port is not None
        self.ser = self._connect_to_device(serial_port)
        rosys.on_startup(self._run)

    @property
    def is_connected(self) -> bool:
        if self.ser is None:
            self.log.debug('Device not connected')
            return False
        if not self.ser.isOpen():
            self.log.debug('Device not open')
            return False
        return True

    async def _run(self) -> None:
        buffer = ''
        last_raw_latitude = 0.0
        last_raw_longitude = 0.0
        last_raw_heading = 0.0
        last_latitude_accuracy = 0.0
        last_longitude_accuracy = 0.0
        last_heading_accuracy = 0.0
        last_gps_qual = 0
        last_num_satellites = 0
        last_hdop = 0.0
        last_altitude = 0.0
        last_gga_timestamp = ''
        last_gst_timestamp = ''
        last_pssn_timestamp = ''
        while True:
            if not self.is_connected:
                return None
            result = await io_bound(self.ser.read_until, b'\r\n')
            if not result:
                self.log.debug('No data')
                continue
            line = result.decode('utf-8')
            if line.endswith('\r\n'):
                sentence = buffer + line.split('*')[0]
                buffer = ''
            else:
                buffer += line
                continue

            self.log.debug(sentence)
            parts = sentence.split(',')
            try:
                timestamp = parts[{'$GPGGA': 1, '$GPGST': 1, '$PSSN': 2}[parts[0]]]
                if parts[0] == '$GPGGA':
                    if parts[2] == '' or parts[4] == '':
                        continue
                    last_gga_timestamp = timestamp
                    last_raw_latitude = self._convert_to_decimal(parts[2], parts[3])
                    last_raw_longitude = self._convert_to_decimal(parts[4], parts[5])
                    last_gps_qual = int(parts[6]) if parts[6] else 0
                    last_num_satellites = int(parts[7]) if parts[7] else 0
                    last_hdop = float(parts[8]) if parts[8] else 0.0
                    last_altitude = float(parts[9]) if parts[9] else 0.0
                if parts[0] == '$GPGST' and parts[6] and parts[7]:
                    last_gst_timestamp = timestamp
                    last_latitude_accuracy = float(parts[6])
                    last_longitude_accuracy = float(parts[7])
                if parts[0] == '$PSSN' and parts[1] == 'HRP':
                    last_pssn_timestamp = timestamp
                    last_raw_heading = float(parts[4] or 0.0)
                    last_heading_accuracy = float(parts[7] or 'inf')
                if last_gga_timestamp == last_gst_timestamp == last_pssn_timestamp != '':
                    antenna = GeoPoint.from_degrees(last_raw_latitude, last_raw_longitude)
                    robot = antenna.polar(self._antenna_distance, -self._antenna_angle + math.radians(last_raw_heading))
                    last_latitude = math.degrees(robot.lat)
                    last_longitude = math.degrees(robot.lon)
                    last_heading = last_raw_heading - self.antenna_pose.yaw
                    self.last_measurement = GnssMeasurement(
                        time=self._gnss_time_to_unix(float(timestamp), rosys.time()),
                        pose=GeoPose.from_degrees(lat=last_latitude, lon=last_longitude, heading=last_heading),
                        latitude_std_dev=last_latitude_accuracy,
                        longitude_std_dev=last_longitude_accuracy,
                        heading_std_dev=last_heading_accuracy,
                        gps_qual=GpsQuality(last_gps_qual),
                        num_satellites=last_num_satellites,
                        hdop=last_hdop,
                        altitude=last_altitude,
                    )
                    self.NEW_MEASUREMENT.emit(self.last_measurement)
            except Exception as e:
                self.log.exception(e)

    # TODO: move to helper and add search argument; for other serial devices
    def _find_device_port(self) -> str | None:
        for port in list_ports.comports():
            self.log.debug('Found port: %s - %s', port.device, port.description)
            if 'Septentrio' in port.description:
                self.log.info('Found GNSS device: %s', port.device)
                return port.device
        raise RuntimeError('No GNSS device found')

    def _connect_to_device(self, port: str, *, baudrate: int = 115200, timeout: float = 0.2) -> serial.Serial:
        self.log.info('Connecting to GNSS device "%s"...', port)
        try:
            return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as e:
            raise RuntimeError(f'Could not connect to GNSS device: {port}') from e

    @staticmethod
    def _convert_to_decimal(coord: str, direction: str) -> float:
        split_index = coord.find('.') - 2
        degrees = float(coord[:split_index])
        minutes = float(coord[split_index:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    @staticmethod
    def _gnss_time_to_unix(gnss_time: float, reference_time: float) -> float:
        """
        Convert GNSS time (HHMMSS.ss) to Unix time, matching the current day from reference_time.

        :param gnss_time: GNSS time in HHMMSS.ss format (e.g., 123456.78 for 12:34:56.78)
        :param reference_time: Reference Unix timestamp (e.g., from rosys.time())
        :return: Unix timestamp for the GNSS time on the same day as reference_time
        """
        hours = int(gnss_time // 10000)
        minutes = int((gnss_time % 10000) // 100)
        seconds = gnss_time % 100
        seconds_since_midnight = hours * 3600 + minutes * 60 + seconds
        day_start = reference_time - (reference_time % 86400)
        return day_start + seconds_since_midnight


class GnssSimulation(Gnss):

    def __init__(self, *, wheels: WheelsSimulation) -> None:
        super().__init__()
        self.wheels = wheels
        self._is_connected = True
        self.last_measurement = GnssMeasurement(
            time=rosys.time(),
            pose=GeoPose.from_pose(self.wheels.pose),
            latitude_std_dev=0.01,
            longitude_std_dev=0.01,
            heading_std_dev=0.1,
            gps_qual=GpsQuality(4),
        )
        rosys.on_repeat(self.simulate, 1.0)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    def simulate(self) -> None:
        if not self.is_connected:
            return
        geo_pose = GeoPose.from_pose(self.wheels.pose)
        self.last_measurement = GnssMeasurement(
            time=rosys.time(),
            pose=geo_pose,
            latitude_std_dev=self.last_measurement.latitude_std_dev,
            longitude_std_dev=self.last_measurement.longitude_std_dev,
            heading_std_dev=self.last_measurement.heading_std_dev,
            gps_qual=self.last_measurement.gps_qual,
        )
        self.NEW_MEASUREMENT.emit(self.last_measurement)

    def developer_ui(self) -> None:
        super().developer_ui()
        ui.label('Simulation').classes('text-center text-bold')
        with ui.column().classes('gap-y-1'):
            ui.checkbox('Connected').bind_value(self, '_is_connected')
            ui.select({quality: quality.name for quality in GpsQuality}, value=GpsQuality.RTK_FIXED,
                      label='Quality').bind_value(self.last_measurement, 'gps_qual').classes('w-4/5')

            ui.number(label='Latitude Std Dev', value=0.01, format='%.3f', prefix='± ', suffix='m').bind_value(
                self.last_measurement, 'latitude_std_dev').classes('w-4/5')
            ui.number(label='Longitude Std Dev', value=0.01, format='%.3f', prefix='± ', suffix='m').bind_value(
                self.last_measurement, 'longitude_std_dev').classes('w-4/5')
            ui.number(label='Heading Std Dev', value=0.1, format='%.2f', prefix='± ', suffix='°').bind_value(
                self.last_measurement, 'heading_std_dev').classes('w-4/5')
