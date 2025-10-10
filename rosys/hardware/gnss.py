from __future__ import annotations

import logging
import math
import re
from abc import ABC
from collections import deque
from dataclasses import dataclass
from datetime import UTC, datetime
from enum import IntEnum
from typing import ClassVar

import numpy as np
import serial
from nicegui import ui
from serial.tools import list_ports

from .. import rosys
from ..driving.driver import PoseProvider
from ..event import Event
from ..geometry import GeoPoint, GeoPose, Pose

SECONDS_DAY = 86400
SECONDS_HALF_DAY = 43200


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


class GnssHardware(Gnss):
    """This hardware module connects to a Septentrio SimpleRTK3b (Mosaic-H) GNSS receiver."""

    # Maximum allowed timestamp difference in seconds (default is ok for Kalman Filter with about 1 km/h)
    MAX_TIMESTAMP_DIFF = 0.05
    MAX_BUFFER_LENGTH = 2000
    NMEA_TYPES: ClassVar[set[str]] = {'GPGGA', 'GPGST', 'PSSN,HRP'}
    NMEA_PATTERN = re.compile(r'\$(?P<type>[A-Z,]+),(?P<timestamp>\d{6}(?:\.\d+)?)[^*]*\*[0-9A-Fa-f]{2}\r\n')

    def __init__(self, *, antenna_pose: Pose | None, reconnect_interval: float = 3.0) -> None:
        """
        :param antenna_pose: the pose of the main antenna in the robot's coordinate frame (yaw: direction to the auxiliary antenna)
        :param reconnect_interval: the interval to wait before reconnecting to the device
        """
        super().__init__()
        self.antenna_pose = antenna_pose or Pose(x=0.0, y=0.0, yaw=0.0)
        self._reconnect_interval = reconnect_interval
        self.serial_connection: serial.Serial | None = None
        rosys.on_startup(self._run)
        self.log.setLevel(logging.DEBUG)
        # TODO: just for evaluation, remove later
        self.diffs: deque[float] = deque(maxlen=10 * 10)

    @property
    def is_connected(self) -> bool:
        if self.serial_connection is None:
            self.log.debug('Device not connected')
            return False
        if not self.serial_connection.isOpen():
            self.log.debug('Device not open')
            return False
        return True

    async def _run(self) -> None:
        buffer = ''
        latest_messages: dict[str, tuple[str, str]] = {}  # type -> (timestamp, sentence)
        while True:
            await rosys.sleep(0)
            if not self.is_connected and not await self._connect():
                continue
            assert self.serial_connection is not None
            result = self.serial_connection.read_all().decode('utf-8', errors='replace')
            if not result:
                continue
            buffer += result
            matches = list(self.NMEA_PATTERN.finditer(buffer))
            for match in reversed(matches):
                self.log.debug('match: %s', match)
                type_, nmea_timestamp = match['type'], match['timestamp']
                if type_ not in self.NMEA_TYPES:
                    continue
                sentence = match.group(0)
                sentence = sentence[:sentence.find('*')]
                latest_messages[type_] = (nmea_timestamp, sentence)
                buffer = buffer[:match.start()]
                if not self.NMEA_TYPES.issubset(latest_messages.keys()):
                    continue
                timestamps = {latest_messages[msg_type][0] for msg_type in self.NMEA_TYPES}
                latest_timestamp = max(timestamps)
                if len(timestamps) != 1:
                    latest_messages = {msg_type: (timestamp, sentence)
                                       for msg_type, (timestamp, sentence) in latest_messages.items()
                                       if timestamp >= latest_timestamp}
                    continue
                if not self.NMEA_TYPES.issubset(latest_messages.keys()):
                    continue
                self.log.debug('found complete trio: %s', nmea_timestamp)
                measurement = self._parse_measurement(latest_messages['GPGGA'][1],
                                                      latest_messages['GPGST'][1],
                                                      latest_messages['PSSN,HRP'][1])
                if measurement is None:
                    continue
                buffer = ''
                latest_messages.clear()
                diff = round(((measurement.gnss_time - rosys.time() + SECONDS_HALF_DAY) %
                              SECONDS_DAY) - SECONDS_HALF_DAY, 3)
                # TODO: just for evaluation, remove later
                self.diffs.append(diff)
                if abs(diff) > self.MAX_TIMESTAMP_DIFF:
                    self.log.warning('timestamp diff = %s (exceeds threshold of %s)', diff, self.MAX_TIMESTAMP_DIFF)
                    continue
                self.log.debug('dt: %s - %s', diff, measurement)
                self.last_measurement = measurement
                self.NEW_MEASUREMENT.emit(measurement)

    async def _connect(self) -> bool:
        try:
            serial_device_path = self._find_device()
            self.serial_connection = self._connect_to_device(serial_device_path)
        except RuntimeError:
            self.log.error('Could not connect to GNSS device: %s', serial_device_path)
            await rosys.sleep(self._reconnect_interval)
            return False
        self.log.info('Connected to GNSS device: %s', serial_device_path)
        # NOTE: Allow time for the device to stabilize after connection
        await rosys.sleep(0.1)
        assert self.serial_connection is not None
        self.serial_connection.reset_input_buffer()
        return True

    def _parse_measurement(self, gga_msg: str, gst_msg: str, pssn_msg: str) -> GnssMeasurement | None:
        gga = Gga.from_sentence(gga_msg)
        if gga is None:
            self.log.debug('Failed to parse GGA: %s', gga_msg)
            return None
        gst = Gst.from_sentence(gst_msg)
        if gst is None:
            self.log.debug('Failed to parse GST: %s', gst_msg)
            return None
        pssn = Pssn.from_sentence(pssn_msg)
        if pssn is None:
            self.log.debug('Failed to parse PSSN: %s', pssn_msg)
            return None
        last_heading = pssn.heading + self.antenna_pose.yaw_deg
        antenna_pose = GeoPose.from_degrees(gga.latitude, gga.longitude, last_heading)
        robot_pose = antenna_pose.relative_shift_by(x=-self.antenna_pose.x, y=-self.antenna_pose.y)
        return GnssMeasurement(time=rosys.time(),
                               gnss_time=gga.timestamp,
                               pose=robot_pose,
                               latitude_std_dev=gst.latitude_std_dev,
                               longitude_std_dev=gst.longitude_std_dev,
                               heading_std_dev=pssn.heading_std_dev,
                               gps_quality=GpsQuality(gga.gps_quality),
                               num_satellites=gga.num_satellites,
                               hdop=gga.hdop,
                               altitude=gga.altitude)

    # TODO: move to helper and add search argument; for other serial devices
    def _find_device(self) -> str:
        for port in list_ports.comports():
            self.log.debug('Found port: %s - %s', port.device, port.description)
            if 'Septentrio' in port.description:
                self.log.debug('Found GNSS device: %s', port.device)
                return port.device
        raise RuntimeError('No GNSS device found')

    def _connect_to_device(self, port: str, *, baudrate: int = 921600, timeout: float = 0.2) -> serial.Serial:
        self.log.debug('Connecting to GNSS device "%s"...', port)
        try:
            return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as e:
            raise RuntimeError(f'Could not connect to GNSS device: {port}') from e


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


@dataclass(slots=True, kw_only=True)
class Gga:
    """
    GPS fix data and undulation

    https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
    """
    timestamp: float
    latitude: float
    longitude: float
    gps_quality: GpsQuality
    num_satellites: int
    hdop: float
    altitude: float
    undulation: float
    correction_age: float

    @staticmethod
    def from_sentence(sentence: str) -> Gga | None:
        parts = sentence.split(',')
        if parts[2] == '' or parts[4] == '':  # latitude or longitude is missing
            return None
        nmea_timestamp = parts[1]
        if not nmea_timestamp:
            return None
        return Gga(timestamp=timestamp_from_nmea(nmea_timestamp),
                   latitude=convert_to_decimal(parts[2], parts[3]),
                   longitude=convert_to_decimal(parts[4], parts[5]),
                   gps_quality=GpsQuality(int(parts[6]) if parts[6] else 0),
                   num_satellites=int(parts[7]) if parts[7] else 0,
                   hdop=float(parts[8]) if parts[8] else 0.0,
                   altitude=float(parts[9]) if parts[9] else 0.0,
                   undulation=float(parts[11]) if parts[11] else 0.0,
                   correction_age=float(parts[13]) if parts[13] else 0.0)


@dataclass(slots=True, kw_only=True)
class Gst:
    """
    Estimated error in position solution

    https://docs.novatel.com/OEM7/Content/Logs/GPGST.htm
    """
    timestamp: float
    semimajor_axis_std_dev: float
    semiminor_axis_std_dev: float
    semi_major_axis_heading: float
    latitude_std_dev: float
    longitude_std_dev: float
    altitude_std_dev: float

    @staticmethod
    def from_sentence(sentence: str) -> Gst | None:
        parts = sentence.split(',')
        if parts[6] == '' or parts[7] == '':
            return None
        return Gst(timestamp=timestamp_from_nmea(parts[1]),
                   semimajor_axis_std_dev=float(parts[3]),
                   semiminor_axis_std_dev=float(parts[4]),
                   semi_major_axis_heading=float(parts[5]),
                   latitude_std_dev=float(parts[6]),
                   longitude_std_dev=float(parts[7]),
                   altitude_std_dev=float(parts[8]))


@dataclass(slots=True, kw_only=True)
class Pssn:
    """PSSN,HRP Septentrio Proprietary Sentence - Heading, Roll, Pitch"""
    timestamp: float
    heading: float
    roll: float
    pitch: float
    heading_std_dev: float
    roll_std_dev: float
    pitch_std_dev: float
    num_satellites: int
    mode: int

    @staticmethod
    def from_sentence(sentence: str) -> Pssn | None:
        parts = sentence.split(',')
        return Pssn(timestamp=timestamp_from_nmea(parts[2]),
                    heading=float(parts[4] or 0.0),
                    roll=float(parts[5] or 0.0),
                    pitch=float(parts[6] or 0.0),
                    heading_std_dev=float(parts[7] or 'inf'),
                    roll_std_dev=float(parts[8] or 'inf'),
                    pitch_std_dev=float(parts[9] or 'inf'),
                    num_satellites=int(parts[10] or 0),
                    mode=int(parts[11] or 0))


def timestamp_from_nmea(nmea_timestamp: str) -> float:
    today = datetime.now().date()
    time_obj = datetime.strptime(nmea_timestamp, '%H%M%S.%f').time()
    utc_time = datetime.combine(today, time_obj).replace(tzinfo=UTC)
    return utc_time.timestamp()


def convert_to_decimal(coord: str, direction: str) -> float:
    """Convert a coordinate in the format DDMM.mmmm to decimal degrees.

    :param coord: the coordinate to convert
    :param direction: the direction (N/S/E/W)
    :return: the coordinate in decimal degrees
    """
    split_index = coord.find('.') - 2
    degrees = float(coord[:split_index])
    minutes = float(coord[split_index:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal
