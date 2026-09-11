import os
import pty
from collections.abc import Generator

import pytest
import serial

from rosys.hardware import GnssHardware
from rosys.testing import forward

SENTENCES = ('$GPGGA,120000.00,4807.03800,N,01131.00000,E,4,12,0.9,545.4,M,46.9,M,,*47\r\n'
             '$GPGST,120000.00,0.01,0.02,0.01,45.0,0.011,0.012,0.02*6A\r\n'
             '$PSSN,HRP,120000.00,110926,90.0,0.0,0.0,0.1,0.1,0.1,12,4,*52\r\n')
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
