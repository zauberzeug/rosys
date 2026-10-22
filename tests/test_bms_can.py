from typing import cast

import pytest

import rosys
from rosys.hardware import BmsCanHardware, CanHardware, RobotBrain
from rosys.testing import forward

# NOTE: lines printed by Lizard for two battery ESPs on a bench, both relays closed
HEARTBEAT_AB58 = 'can 1f10ab58,07,1b,ff,ff,40,0a,00,00'
BMS_1_AB58 = 'can 1f11ab58,bf,09,b8,0b,00,00,55,03'
BMS_2_AB58 = 'can 1f12ab58,9c,0b,90,0b,87,0b,ff,ff'
BMS_FIXED_AB58 = 'can 1f13ab58,01,00,6c,34,24,08,03,ff'
IDENTITY_AB58 = 'can 1f18ab58,58,ab,e8,4e,b5,80,01,00'
DIAGNOSTICS_AB58 = 'can 1f19ab58,01,00,00,00,00,00,00,00'
CELLS_AB58 = ['can 1f21ab58,00,08,00,00,00,00,ff,ff',
              'can 1f21ab58,01,d1,0c,d1,0c,d1,0c',
              'can 1f21ab58,02,d1,0c,d2,0c,d0,0c',
              'can 1f21ab58,03,d3,0c,cf,0c,ff,ff']
HEARTBEAT_A8F4 = 'can 1f10a8f4,07,1d,ff,ff,41,0a,00,00'
BMS_1_A8F4 = 'can 1f11a8f4,0b,09,b8,0b,00,00,4d,03'
BMS_2_A8F4 = 'can 1f12a8f4,9d,0b,82,0b,86,0b,ff,ff'


class RobotBrainStub:

    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send(self, msg: str) -> None:
        self.sent.append(msg)


@pytest.fixture
def bms(rosys_integration: None) -> BmsCanHardware:
    robot_brain = cast(RobotBrain, RobotBrainStub())
    can = CanHardware(robot_brain, baud=500_000)
    return BmsCanHardware(robot_brain, can=can)


def receive(bms: BmsCanHardware, *lines: str) -> None:
    for line in lines:
        bms.message_hooks['can'](line)


def sent(bms: BmsCanHardware) -> list[str]:
    return cast(RobotBrainStub, bms.robot_brain).sent


def test_lizard_code(bms: BmsCanHardware) -> None:
    assert bms.lizard_code == 'can.output_extended(0x1f000000, 0x1f000000)'


def test_one_pack(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, BMS_2_AB58, BMS_FIXED_AB58, IDENTITY_AB58, DIAGNOSTICS_AB58, *CELLS_AB58)
    pack = bms.packs[0xAB58]
    assert pack.relay_on and pack.bus_voltage and pack.bms_valid
    assert not pack.load_free_valid and not pack.error_active and not pack.shut_down and not pack.measuring
    assert pack.protocol_version == 1
    assert pack.load_free_voltage is None
    assert pack.voltage == pytest.approx(26.24)
    assert pack.current == 0
    assert pack.remaining_capacity == pytest.approx(24.95)
    assert pack.nominal_capacity == pytest.approx(30.0)
    assert pack.protection_flags == 0
    assert pack.percentage == 85
    assert pack.fet_state == 3
    assert pack.temperatures == pytest.approx([24.05, 22.85, 21.95], abs=0.1)
    assert pack.cycles == 1
    assert pack.production_date == '2026-03-12'
    assert pack.cell_count == 8
    assert pack.sensor_count == 3
    assert pack.chip_id == '80:b5:4e:e8:ab:58'
    assert pack.firmware_version == '0.1'
    assert pack.reset_reason == 1
    assert pack.restart_count == 0
    assert pack.cell_voltages == pytest.approx([3.281, 3.281, 3.281, 3.281, 3.282, 3.280, 3.283, 3.279])
    assert pack.balancing == 0


async def test_combined_state(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, BMS_2_AB58, HEARTBEAT_A8F4, BMS_1_A8F4, BMS_2_A8F4)
    await forward(seconds=1.5)
    assert bms.state.percentage == pytest.approx(100 * (24.95 + 23.15) / 60.0)
    assert bms.state.voltage == pytest.approx((26.24 + 26.25) / 2)
    assert bms.state.current == 0
    assert bms.state.temperature == pytest.approx(24.15, abs=0.1)
    assert bms.state.is_charging


async def test_voltage_only_from_connected_packs(bms: BmsCanHardware) -> None:
    relay_off = 'can 1f10a8f4,0c,1d,41,0a,41,0a,00,00'
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, relay_off, BMS_1_A8F4)
    await forward(seconds=1.5)
    assert bms.state.voltage == pytest.approx(26.24)
    assert bms.packs[0xA8F4].load_free_voltage == pytest.approx(26.25)


async def test_summed_current_decides_charging(bms: BmsCanHardware) -> None:
    discharging = 'can 1f10ab58,07,1b,ff,ff,40,0a,ce,ff'  # -0.50 A
    charging = 'can 1f10a8f4,07,1d,ff,ff,41,0a,c8,00'  # +2.00 A
    receive(bms, discharging, BMS_1_AB58, HEARTBEAT_A8F4, BMS_1_A8F4)
    await forward(seconds=1.5)
    assert bms.state.current == pytest.approx(-0.5)
    assert not bms.state.is_charging
    receive(bms, discharging, charging)
    await forward(seconds=1.0)
    assert bms.state.current == pytest.approx(1.5)
    assert bms.state.is_charging


async def test_missing_pack_is_dropped(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, HEARTBEAT_A8F4, BMS_1_A8F4)
    for _ in range(4):
        await forward(seconds=1.0)
        receive(bms, HEARTBEAT_AB58)
    assert [pack.node_id for pack in bms.fresh_packs] == [0xAB58]
    assert bms.state.percentage == pytest.approx(100 * 24.95 / 30.0)


async def test_commands(bms: BmsCanHardware) -> None:
    await bms.start_measurement_phase()
    await bms.request_cell_voltages(0xA8F4)
    assert sent(bms) == ['can.send(0x1f080000, 0, 0, 0, 0, 0, 0, 0, 0)',
                         'can.send(0x1f20a8f4, 0, 0, 0, 0, 0, 0, 0, 0)']


async def test_shutdown_is_repeated_and_reports_remaining_relays(bms: BmsCanHardware) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    rosys.background_tasks.create(bms.shut_down(), name='shut_down')
    for _ in range(25):
        receive(bms, HEARTBEAT_AB58)
        await forward(seconds=0.1)
    assert sent(bms)[0] == 'can.send(0x1f000000, 65, 75, 75, 85, 45, 65, 85, 83)'
    assert len(sent(bms)) >= 20
    assert any('ab58' in n for n in notifications)


async def test_automatic_measurement_phase(rosys_integration: None) -> None:
    robot_brain = cast(RobotBrain, RobotBrainStub())
    load_low = False
    bms = BmsCanHardware(robot_brain, can=CanHardware(robot_brain), is_load_low=lambda: load_low)
    relay_off = 'can 1f10a8f4,0c,1d,41,0a,41,0a,00,00'
    for _ in range(3):
        receive(bms, HEARTBEAT_AB58, BMS_1_AB58, relay_off, BMS_1_A8F4)
        await forward(seconds=1.0)
    assert not sent(bms)
    load_low = True
    for _ in range(3):
        receive(bms, HEARTBEAT_AB58, BMS_1_AB58, relay_off, BMS_1_A8F4)
        await forward(seconds=1.0)
    assert sent(bms) == ['can.send(0x1f080000, 0, 0, 0, 0, 0, 0, 0, 0)']


def test_error_frame(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_A8F4, 'can 1f01a8f4,02,f4,a8,88,13,00,00')
    assert bms.packs[0xA8F4].last_message == \
        'battery a8f4: BMS does not answer, relay off (code 0x02, node a8f4, detail 5000)'


def test_ignores_other_frames(bms: BmsCanHardware) -> None:
    receive(bms, 'can 181,01,02', 'can 1e10ab58,07,1b,ff,ff,40,0a,00,00', 'can 1f0cab58,40,0a', 'can 1f10ab58,07')
    assert not bms.packs
