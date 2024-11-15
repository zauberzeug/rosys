from __future__ import annotations

import logging
import math
from abc import ABC
from dataclasses import dataclass
from typing import TYPE_CHECKING

import serial
from nicegui import ui
from serial.tools import list_ports

from .. import rosys
from ..event import Event
from ..geometry.geo import GeoPoint, GeoPose, GeoReference
from ..run import io_bound

if TYPE_CHECKING:
    from ..hardware import WheelsSimulation


@dataclass
class GnssMeasurement:
    time: float
    # TODO: back to GeoPoint and Heading
    location: GeoPose
    latitude_std_dev: float = 0.0
    longitude_std_dev: float = 0.0
    heading_std_dev: float = 0.0
    # TODO
    mode: str = ''
    # TODO
    gps_qual: int = 0
    # TODO
    altitude: float = 0.0
    # TODO
    separation: float = 0.0


class Gnss(ABC):

    def __init__(self, *, reference: GeoReference | None = None) -> None:
        self.log = logging.getLogger('rosys.gnss')
        self.reference: GeoReference = GeoReference(origin=GeoPoint(
            lat=0, lon=0), direction=0) if reference is None else reference
        self.last_measurement: GnssMeasurement | None = None

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received"""

    @property
    def is_connected(self) -> bool:
        return False

    def update_reference(self, *, reference: GeoReference | None = None) -> None:
        if reference is not None:
            self.reference = reference
        elif self.last_measurement is not None:
            self.reference = GeoReference(origin=self.last_measurement.location.point, direction=0.0)

    @ui.refreshable
    def developer_ui(self) -> None:
        ui.label('GNSS').classes('text-center text-bold')
        ui.label(f'Reference: {self.reference}')
        # TODO:


class GnssHardware(Gnss):
    """
    #TODO:
    """
    ANTENNA_OFFSET = 0.4225  # meters

    def __init__(self, *, antenna_offset: float = ANTENNA_OFFSET) -> None:
        super().__init__()
        self.antenna_offset = antenna_offset
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
                    last_gga_timestamp = timestamp
                    last_raw_latitude = self._convert_to_decimal(parts[2], parts[3])
                    last_raw_longitude = self._convert_to_decimal(parts[4], parts[5])
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
                    robot = antenna.polar(self.ANTENNA_OFFSET, math.radians(last_raw_heading))
                    last_latitude = math.degrees(robot.lat)
                    last_longitude = math.degrees(robot.lon)
                    last_heading = last_raw_heading
                    self.last_measurement = GnssMeasurement(
                        time=timestamp,
                        location=GeoPose.from_degrees(lat=last_latitude, lon=last_longitude, heading=last_heading),
                        latitude_std_dev=last_latitude_accuracy,
                        longitude_std_dev=last_longitude_accuracy,
                        heading_std_dev=last_heading_accuracy,
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
            return serial.Serial(port, baudrate, timeout)
        except serial.SerialException as e:
            raise RuntimeError(f'Could not connect to GNSS device: {port}') from e

    @staticmethod
    def _convert_to_decimal(coord: str, direction: str) -> float:
        degrees = float(coord[:2])
        minutes = float(coord[2:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return round(decimal, 6)


class GnssSimulation(Gnss):

    def __init__(self, wheels: WheelsSimulation) -> None:
        super().__init__()
        self.wheels = wheels
        rosys.on_repeat(self.simulate, 1.0)

    @property
    def is_connected(self) -> bool:
        return True

    def simulate(self) -> None:
        geo_pose = self.reference.pose_to_geo(self.wheels.pose)
        self.last_measurement = GnssMeasurement(
            time=rosys.time(),
            location=geo_pose,
            latitude_std_dev=0.01,
            longitude_std_dev=0.01,
            heading_std_dev=0.1,
            gps_qual=4,
        )
        self.NEW_MEASUREMENT.emit(self.last_measurement)
