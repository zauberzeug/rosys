from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
from enum import IntEnum


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
    def from_sentence(sentence: str) -> Gga:
        parts = sentence.split(',')
        if not parts[1]:
            raise ValueError(f'Missing timestamp in GGA: {sentence}')
        if parts[2] == '' or parts[4] == '':
            raise ValueError(f'Missing latitude or longitude in GGA: {sentence}')
        return Gga(timestamp=timestamp_from_nmea(parts[1]),
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
    def from_sentence(sentence: str) -> Gst:
        parts = sentence.split(',')
        if any(part == '' for part in parts[1:9]):
            raise ValueError(f'Missing data in GST: {sentence}')
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
    def from_sentence(sentence: str) -> Pssn:
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
