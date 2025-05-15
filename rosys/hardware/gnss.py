from __future__ import annotations

import logging
import math
from abc import ABC
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import IntEnum

import numpy as np
import serial
from nicegui import ui
from serial.tools import list_ports

from .. import rosys
from ..driving.driver import PoseProvider
from ..event import Event
from ..geometry import GeoPoint, GeoPose, Pose
from ..run import io_bound

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
        last_raw_latitude = 0.0
        last_raw_longitude = 0.0
        last_raw_heading = 0.0
        last_latitude_accuracy = 0.0
        last_longitude_accuracy = 0.0
        last_heading_accuracy = 0.0
        last_gps_quality = 0
        last_num_satellites = 0
        last_hdop = 0.0
        last_altitude = 0.0
        last_gga_timestamp = 0.0
        last_gst_timestamp = 0.0
        last_pssn_timestamp = 0.0

        while True:
            if not self.is_connected:
                try:
                    serial_device_path = self._find_device()
                    self.serial_connection = self._connect_to_device(serial_device_path)
                except RuntimeError:
                    self.log.error('Could not connect to GNSS device: %s', serial_device_path)
                    await rosys.sleep(self._reconnect_interval)
                    continue
                self.log.info('Connected to GNSS device: %s', serial_device_path)
                # NOTE: Allow time for the device to stabilize after connection
                await rosys.sleep(0.1)
                assert self.serial_connection is not None
                self.serial_connection.reset_input_buffer()

            assert self.serial_connection is not None
            result = await io_bound(self.serial_connection.read_until, b'\r\n')
            rosys_time = rosys.time()
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
                field_index = {'$GPGGA': 1, '$GPGST': 1, '$PSSN': 2}
                if parts[0] not in field_index:
                    continue
                nmea_timestamp = parts[field_index[parts[0]]]
                if not nmea_timestamp:
                    continue
                today = datetime.now().date()
                time_obj = datetime.strptime(nmea_timestamp, '%H%M%S.%f').time()
                utc_time = datetime.combine(today, time_obj).replace(tzinfo=timezone.utc)
                timestamp = utc_time.timestamp()
                diff = round(((timestamp - rosys_time + SECONDS_HALF_DAY) % SECONDS_DAY) - SECONDS_HALF_DAY, 3)
                if abs(diff) > self.MAX_TIMESTAMP_DIFF:
                    self.log.warning('timestamp diff = %s (exceeds threshold of %s)', diff, self.MAX_TIMESTAMP_DIFF)
                    continue
                else:
                    self.log.debug('timestamp diff = %s', diff)
                if parts[0] == '$GPGGA':
                    if parts[2] == '' or parts[4] == '':
                        continue
                    last_gga_timestamp = timestamp
                    last_raw_latitude = self._convert_to_decimal(parts[2], parts[3])
                    last_raw_longitude = self._convert_to_decimal(parts[4], parts[5])
                    last_gps_quality = int(parts[6]) if parts[6] else 0
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
                if last_gga_timestamp == last_gst_timestamp == last_pssn_timestamp:
                    last_heading = last_raw_heading + self.antenna_pose.yaw_deg
                    antenna_pose = GeoPose.from_degrees(last_raw_latitude, last_raw_longitude, last_heading)
                    robot_pose = antenna_pose.relative_shift_by(x=-self.antenna_pose.x, y=-self.antenna_pose.y)
                    self.last_measurement = GnssMeasurement(
                        time=rosys.time(),
                        gnss_time=timestamp,
                        pose=robot_pose,
                        latitude_std_dev=last_latitude_accuracy,
                        longitude_std_dev=last_longitude_accuracy,
                        heading_std_dev=last_heading_accuracy,
                        gps_quality=GpsQuality(last_gps_quality),
                        num_satellites=last_num_satellites,
                        hdop=last_hdop,
                        altitude=last_altitude,
                    )
                    self.NEW_MEASUREMENT.emit(self.last_measurement)
            except (KeyError, ValueError) as e:
                self.log.exception('GNSS parse error: %s', e)

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

    @staticmethod
    def _convert_to_decimal(coord: str, direction: str) -> float:
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


class GnssSimulation(Gnss):
    """Simulation of a GNSS receiver."""

    def __init__(self, *,
                 wheels: PoseProvider,
                 lat_std_dev: float = 0.01,
                 lon_std_dev: float = 0.01,
                 heading_std_dev: float = 0.01,
                 gps_quality: GpsQuality = GpsQuality.RTK_FIXED) -> None:
        """
        :param wheels: the wheels to use for the simulation
        :param lat_std_dev: the standard deviation of the latitude in meters
        :param lon_std_dev: the standard deviation of the longitude in meters
        :param heading_std_dev: the standard deviation of the heading in degrees
        :param gps_quality: the quality of the GPS signal
        """
        super().__init__()
        self.wheels = wheels
        self._is_connected = True
        self._lat_std_dev = lat_std_dev
        self._lon_std_dev = lon_std_dev
        self._heading_std_dev = heading_std_dev
        self._gps_quality = gps_quality
        self.last_measurement: GnssMeasurement | None = None
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
        self.NEW_MEASUREMENT.emit(self.last_measurement)

    def developer_ui(self) -> None:
        super().developer_ui()
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
