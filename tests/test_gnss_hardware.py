import os
import pty
from collections.abc import Generator

import pytest
import serial

from rosys.hardware import GnssHardware
from rosys.hardware.gnss import GnssMeasurement
from rosys.hardware.gnss.nmea import timestamp_from_nmea
from rosys.testing import forward


def gga(timestamp: str) -> str:
    return f'$GPGGA,{timestamp},4807.03800,N,01131.00000,E,4,12,0.9,545.4,M,46.9,M,,*47\r\n'


def gst(timestamp: str) -> str:
    return f'$GPGST,{timestamp},0.01,0.02,0.01,45.0,0.011,0.012,0.02*6A\r\n'


def hrp(timestamp: str) -> str:
    return f'$PSSN,HRP,{timestamp},110926,90.0,0.0,0.0,0.1,0.1,0.1,12,4,*52\r\n'


def sentences(timestamp: str) -> str:
    return gga(timestamp) + gst(timestamp) + hrp(timestamp)


SENTENCES = sentences('120000.00')
SENTENCES_WITHOUT_TIME = ('$GPGGA,,,,,,0,00,,,M,,M,,*66\r\n'
                          '$PSSN,HRP,,,,,,,,,00,0,,*48\r\n'
                          '$GPGST,,,,,,,,*57\r\n')


@pytest.fixture
def receiver(rosys_integration: None) -> Generator[tuple[GnssHardware, int], None, None]:
    controller, device = pty.openpty()
    gnss = GnssHardware(antenna_pose=None, max_measurement_age=float('inf'))
    gnss.serial_connection = serial.Serial(os.ttyname(device))
    yield gnss, controller
    gnss.serial_connection.close()
    os.close(controller)
    os.close(device)


async def test_measurement_from_a_complete_sentence_set(receiver: tuple[GnssHardware, int]) -> None:
    gnss, controller = receiver
    os.write(controller, SENTENCES.encode())
    await forward(seconds=0.1)
    assert gnss.last_measurement is not None
    assert gnss.last_measurement.pose.degree_tuple[0] == pytest.approx(48.1173)
    assert gnss.last_measurement.gps_quality == 4


async def test_measurement_from_sentences_split_across_reads(receiver: tuple[GnssHardware, int]) -> None:
    gnss, controller = receiver
    split = SENTENCES.index('$GPGST') + 20
    os.write(controller, SENTENCES[:split].encode())
    await forward(seconds=0.1)
    os.write(controller, SENTENCES[split:].encode())
    await forward(seconds=0.1)
    assert gnss.last_measurement is not None


async def test_measurement_after_a_receiver_without_time(receiver: tuple[GnssHardware, int]) -> None:
    gnss, controller = receiver
    for _ in range(100):
        os.write(controller, SENTENCES_WITHOUT_TIME.encode())
        await forward(seconds=0.1)
    assert gnss.last_measurement is None
    os.write(controller, SENTENCES.encode())
    await forward(seconds=0.1)
    assert gnss.last_measurement is not None


@pytest.mark.parametrize(('reads', 'expected_gnss_times'), [
    pytest.param([sentences('120000.00') + sentences('120001.00')], ['120001.00'],
                 id='newest set in a read wins'),
    pytest.param([gga('120001.00') + hrp('120001.00') + gst('120000.00')], [],
                 id='mismatched timestamps do not emit'),
    pytest.param([sentences('120000.00'), gga('120000.00'), gst('120000.00')], ['120000.00'],
                 id='repeated sentence does not re-emit'),
    pytest.param([sentences('120000.00'), sentences('120001.00')], ['120000.00', '120001.00'],
                 id='consecutive sets emit once each'),
])
async def test_which_sentence_sets_are_emitted(receiver: tuple[GnssHardware, int],
                                               reads: list[str], expected_gnss_times: list[str]) -> None:
    gnss, controller = receiver
    measurements: list[GnssMeasurement] = []
    gnss.NEW_MEASUREMENT.subscribe(measurements.append)
    for read in reads:
        os.write(controller, read.encode())
        await forward(seconds=0.1)
    assert [m.gnss_time for m in measurements] == [timestamp_from_nmea(t) for t in expected_gnss_times]
