import re
from typing import ClassVar

import serial
from serial.tools import list_ports

from ... import rosys
from ...geometry import GeoPose, Pose
from .gnss import Gnss, GnssMeasurement
from .nmea import Gga, GpsQuality, Gst, Pssn


class GnssHardware(Gnss):
    """This hardware module connects to a Septentrio SimpleRTK3b (Mosaic-H) GNSS receiver."""
    MAX_MEASUREMENT_AGE = 0.05
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
            await rosys.sleep(0.01)
            if not self.is_connected and not await self._connect():
                continue
            assert self.serial_connection is not None
            result = self.serial_connection.read_all().decode('utf-8', errors='replace')
            if not result:
                continue
            buffer += result
            matches = list(self.NMEA_PATTERN.finditer(buffer))
            for match in reversed(matches):
                type_, nmea_timestamp = match['type'], match['timestamp']
                if type_ not in self.NMEA_TYPES:
                    self.log.debug('Skipping unknown type: %s', type_)
                    continue
                sentence = match.group(0)
                self.log.debug('%s, %s: %s', type_, nmea_timestamp, sentence)
                sentence = sentence[:sentence.find('*')]
                latest_messages[type_] = (nmea_timestamp, sentence)
                buffer = buffer[:match.start()]
                if not self.NMEA_TYPES.issubset(latest_messages):
                    continue
                timestamps = {latest_messages[msg_type][0] for msg_type in self.NMEA_TYPES}
                if len(timestamps) != 1:
                    latest_timestamp = max(timestamps)
                    latest_messages = {msg_type: (timestamp, sentence)
                                       for msg_type, (timestamp, sentence) in latest_messages.items()
                                       if timestamp >= latest_timestamp}
                    continue
                if not self.NMEA_TYPES.issubset(latest_messages):
                    continue
                try:
                    measurement = self._parse_measurement(latest_messages['GPGGA'][1],
                                                          latest_messages['GPGST'][1],
                                                          latest_messages['PSSN,HRP'][1])
                except ValueError as e:
                    self.log.debug('Failed to parse measurement: %s', e)
                    continue
                measurement_age = measurement.age
                if abs(measurement_age) > self.MAX_MEASUREMENT_AGE:
                    self.log.warning('measurement age = %.3f (exceeds threshold of %s)',
                                     measurement_age, self.MAX_MEASUREMENT_AGE)
                    continue
                self.log.debug('dt: %.3f - %s', measurement_age, measurement)
                self.last_measurement = measurement
                self.NEW_MEASUREMENT.emit(measurement)
                buffer = ''
                latest_messages.clear()
                break

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

    def _parse_measurement(self, gga_msg: str, gst_msg: str, pssn_msg: str) -> GnssMeasurement:
        gga = Gga.from_sentence(gga_msg)
        gst = Gst.from_sentence(gst_msg)
        pssn = Pssn.from_sentence(pssn_msg)
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
