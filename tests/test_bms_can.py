from typing import cast

import pytest

import rosys
from rosys.hardware import BmsCanHardware, BmsCanSimulation, CanHardware, EspNotReadyException, RobotBrain
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
RELAY_OFF_A8F4 = 'can 1f10a8f4,0c,1d,41,0a,41,0a,00,00'
ERROR_A8F4 = 'can 1f10a8f4,14,1d,ff,ff,41,0a,00,00'
ONE_PACK_OFF = (HEARTBEAT_AB58, BMS_1_AB58, RELAY_OFF_A8F4, BMS_1_A8F4)


class RobotBrainStub:

    def __init__(self) -> None:
        self.sent: list[str] = []
        self.is_ready = True

    async def send(self, msg: str) -> None:
        if not self.is_ready:
            raise EspNotReadyException('Sending message failed because ESP is not ready')
        self.sent.append(msg)


def create_bms(**kwargs) -> BmsCanHardware:
    robot_brain = cast(RobotBrain, RobotBrainStub())
    return BmsCanHardware(robot_brain, can=CanHardware(robot_brain, baud=500_000), **kwargs)


@pytest.fixture
def bms(rosys_integration: None) -> BmsCanHardware:
    return create_bms()


def receive(bms: BmsCanHardware, *lines: str) -> None:
    for line in lines:
        bms.message_hooks['can'](line)


async def receive_every_second(bms: BmsCanHardware, seconds: int, *lines: str) -> None:
    for _ in range(seconds):
        receive(bms, *lines)
        await forward(seconds=1.0)


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


def test_new_cell_request_drops_old_voltages(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, *CELLS_AB58)
    receive(bms, *CELLS_AB58[:2])
    assert bms.packs[0xAB58].cell_voltages == pytest.approx([3.281, 3.281, 3.281, None, None, None, None, None])


def test_error_frame(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_A8F4, 'can 1f01a8f4,02,f4,a8,88,13,00,00')
    assert bms.packs[0xA8F4].last_message == \
        'battery a8f4: BMS does not answer, relay off (code 0x02, node a8f4, detail 5000)'


def test_ignores_other_frames(bms: BmsCanHardware) -> None:
    receive(bms, 'can 181,01,02', 'can 1e10ab58,07,1b,ff,ff,40,0a,00,00', 'can 1f0cab58,40,0a', 'can 1f10ab58,07')
    assert not bms.packs


async def test_combined_state(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, BMS_2_AB58, HEARTBEAT_A8F4, BMS_1_A8F4, BMS_2_A8F4)
    await forward(seconds=1.5)
    assert bms.state.percentage == pytest.approx(100 * (24.95 + 23.15) / 60.0)
    assert bms.state.voltage == pytest.approx((26.24 + 26.25) / 2)
    assert bms.state.current == 0
    assert bms.state.temperature == pytest.approx(24.15, abs=0.1)
    assert bms.state.is_charging


async def test_voltage_only_from_connected_packs(bms: BmsCanHardware) -> None:
    receive(bms, *ONE_PACK_OFF)
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


async def test_percentage_ignores_packs_off_with_error(bms: BmsCanHardware) -> None:
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, ERROR_A8F4, BMS_1_A8F4)
    await forward(seconds=1.5)
    assert bms.state.percentage == pytest.approx(100 * 24.95 / 30.0)


async def test_percentage_unknown_without_nominal_capacity(bms: BmsCanHardware) -> None:
    unknown_nominal = 'can 1f11a8f4,0b,09,ff,ff,00,00,4d,03'
    receive(bms, HEARTBEAT_AB58, BMS_1_AB58, HEARTBEAT_A8F4, unknown_nominal)
    await forward(seconds=1.5)
    assert bms.state.percentage is None


async def test_state_is_cleared_when_all_packs_are_missing(bms: BmsCanHardware) -> None:
    charging = 'can 1f10ab58,07,1b,ff,ff,40,0a,c8,00'
    receive(bms, charging, BMS_1_AB58, BMS_2_AB58)
    await forward(seconds=1.5)
    assert bms.state.is_charging
    await forward(seconds=5.0)
    assert bms.state.percentage is None
    assert bms.state.voltage is None
    assert bms.state.current is None
    assert bms.state.temperature is None
    assert not bms.state.is_charging
    assert bms.state.last_update < bms.PACK_TIMEOUT


async def test_commands(bms: BmsCanHardware) -> None:
    await bms.start_measurement_phase()
    await bms.request_cell_voltages(0xA8F4)
    assert sent(bms) == ['can.send(0x1f080000, 0, 0, 0, 0, 0, 0, 0, 0)',
                         'can.send(0x1f20a8f4, 0, 0, 0, 0, 0, 0, 0, 0)']


async def test_shutdown_is_repeated_and_reports_remaining_relays(bms: BmsCanHardware) -> None:
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    receive(bms, HEARTBEAT_A8F4)
    await forward(seconds=4.0)
    rosys.background_tasks.create(bms.shut_down(), name='shut_down')
    for _ in range(25):
        receive(bms, HEARTBEAT_AB58)
        await forward(seconds=0.1)
    assert sent(bms)[0] == 'can.send(0x1f000000, 65, 75, 75, 85, 45, 65, 85, 83)'
    assert len(sent(bms)) >= 20
    assert 'Battery relays still on: a8f4 (no heartbeat), ab58' in notifications


async def test_shutdown_survives_core_not_ready(bms: BmsCanHardware) -> None:
    robot_brain = cast(RobotBrainStub, bms.robot_brain)
    robot_brain.is_ready = False
    task = rosys.background_tasks.create(bms.shut_down(), name='shut_down')
    await forward(seconds=0.5)
    assert not sent(bms)
    robot_brain.is_ready = True
    await forward(seconds=0.5)
    assert sent(bms)
    assert bms.is_shutting_down
    task.cancel()
    await forward(seconds=0.1)
    assert not bms.is_shutting_down


async def test_automatic_measurement_phase(rosys_integration: None) -> None:
    load_low = False
    bms = create_bms(is_load_low=lambda: load_low)
    await receive_every_second(bms, 3, *ONE_PACK_OFF)
    assert not sent(bms)
    load_low = True
    await receive_every_second(bms, 3, *ONE_PACK_OFF)
    assert sent(bms) == ['can.send(0x1f080000, 0, 0, 0, 0, 0, 0, 0, 0)']
    await receive_every_second(bms, 55, *ONE_PACK_OFF)
    assert len(sent(bms)) == 1, 'no second phase within measurement_phase_interval'
    await receive_every_second(bms, 5, *ONE_PACK_OFF)
    assert len(sent(bms)) == 2


async def test_no_automatic_measurement_phase_while_core_is_not_ready(rosys_integration: None) -> None:
    bms = create_bms(is_load_low=lambda: True)
    robot_brain = cast(RobotBrainStub, bms.robot_brain)
    robot_brain.is_ready = False
    await receive_every_second(bms, 3, *ONE_PACK_OFF)
    assert not sent(bms)
    robot_brain.is_ready = True
    await receive_every_second(bms, 3, *ONE_PACK_OFF)
    assert sent(bms) == ['can.send(0x1f080000, 0, 0, 0, 0, 0, 0, 0, 0)']


@pytest.mark.parametrize('heartbeat', [
    ERROR_A8F4,
    'can 1f10a8f4,24,1d,ff,ff,41,0a,00,00',  # shut down
    'can 1f10a8f4,0c,0d,41,0a,41,0a,00,00',  # protocol version 0 next to version 1
])
async def test_no_automatic_measurement_phase_for_excluded_packs(rosys_integration: None, heartbeat: str) -> None:
    bms = create_bms(is_load_low=lambda: True)
    await receive_every_second(bms, 3, HEARTBEAT_AB58, BMS_1_AB58, heartbeat, BMS_1_A8F4)
    assert not sent(bms)


async def test_simulation(rosys_integration: None) -> None:
    bms = BmsCanSimulation()
    started: list[bool] = []
    bms.MEASUREMENT_PHASE_STARTED.subscribe(lambda: started.append(True))
    await bms.start_measurement_phase()
    assert started
    await bms.request_cell_voltages(0xAB58)
    await bms.shut_down()
    assert bms.is_shutting_down
