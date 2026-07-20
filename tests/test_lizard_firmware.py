import pytest
from nicegui import background_tasks
from packaging.specifiers import InvalidSpecifier
from test_robot_brain import CommunicationSimulation, connect  # pylint: disable=wrong-import-order

import rosys
from rosys.hardware import RobotBrain, RobotHardware
from rosys.hardware.lizard_firmware import LizardFirmware
from rosys.testing import forward


class VersionCommunicationSimulation(CommunicationSimulation):

    def __init__(self) -> None:
        super().__init__()
        self.core_version = 'v0.13.0'
        self.p0_version = 'v0.13.0'

    async def send(self, msg: str) -> None:
        await super().send(msg)
        line = msg.rsplit('@', 1)[0]
        if line == 'core.version()':
            self.incoming.append(f'version: {self.core_version}')
        if line == 'p0.version()':
            self.incoming.append(f'p0: version: {self.p0_version}')


@pytest.fixture
async def lizard_firmware(rosys_integration: None) -> LizardFirmware:
    robot_brain = RobotBrain(VersionCommunicationSimulation(),
                             enable_esp_on_startup=False,
                             supported_lizard_versions='<0.14.0')
    RobotHardware([], robot_brain)
    return robot_brain.lizard_firmware


def test_all_versions_are_supported_without_specifier(lizard_firmware: LizardFirmware) -> None:
    """Configurations without a version restriction accept any version."""
    lizard_firmware.supported_versions = None
    assert lizard_firmware.is_version_supported('0.12.0')
    assert lizard_firmware.is_version_supported('0.14.0')
    assert lizard_firmware.is_version_supported('99.0.0')


def test_upper_bound_specifier(lizard_firmware: LizardFirmware) -> None:
    """An upper bound like ``<0.14.0`` allows older versions and rejects the bound itself."""
    assert lizard_firmware.is_version_supported('0.12.5')
    assert lizard_firmware.is_version_supported('0.13.0')
    assert not lizard_firmware.is_version_supported('0.14.0')
    assert not lizard_firmware.is_version_supported('0.14.1')


def test_exact_version_specifier(lizard_firmware: LizardFirmware) -> None:
    """An exact specifier like ``==0.13.0`` only allows that very version."""
    lizard_firmware.supported_versions = '==0.13.0'
    assert lizard_firmware.is_version_supported('0.13.0')
    assert not lizard_firmware.is_version_supported('0.12.9')
    assert not lizard_firmware.is_version_supported('0.13.1')


def test_combined_specifier(lizard_firmware: LizardFirmware) -> None:
    """Combined bounds like ``>0.12.0,<0.14.0`` are evaluated together."""
    lizard_firmware.supported_versions = '>0.12.0,<0.14.0'
    assert not lizard_firmware.is_version_supported('0.12.0')
    assert lizard_firmware.is_version_supported('0.12.1')
    assert lizard_firmware.is_version_supported('0.13.9')
    assert not lizard_firmware.is_version_supported('0.14.0')


def test_unparsable_versions_are_not_supported(lizard_firmware: LizardFirmware) -> None:
    """With a restriction in place, unparsable versions are rejected."""
    assert not lizard_firmware.is_version_supported('')
    assert not lizard_firmware.is_version_supported('not-a-version')


async def test_invalid_specifier_raises(rosys_integration: None) -> None:
    """An invalid specifier fails fast when constructing the robot brain."""
    with pytest.raises(InvalidSpecifier):
        RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False, supported_lizard_versions='latest')


async def test_online_versions_exclude_unsupported(lizard_firmware: LizardFirmware, monkeypatch: pytest.MonkeyPatch) -> None:
    """Reading online versions drops versions that do not satisfy the specifier."""
    releases = [
        {'tag_name': 'v0.14.0', 'assets': [{'browser_download_url': 'https://example.com/lizard-0.14.0.zip'}]},
        {'tag_name': 'v0.13.0', 'assets': [{'browser_download_url': 'https://example.com/lizard-0.13.0.zip'}]},
        {'tag_name': 'v0.12.0', 'assets': [{'browser_download_url': 'https://example.com/lizard-0.12.0.zip'}]},
    ]

    class FakeResponse:
        def json(self) -> list[dict]:
            return releases
    monkeypatch.setattr('rosys.hardware.lizard_firmware.requests.get', lambda url: FakeResponse())
    await lizard_firmware.read_online_version()
    assert list(lizard_firmware.online_versions) == ['0.13.0', '0.12.0']


async def test_download_refuses_unsupported_version(lizard_firmware: LizardFirmware) -> None:
    """Downloading an unsupported version is refused even if it was selected programmatically."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    lizard_firmware.online_versions = {'0.14.0': 'https://example.com/lizard-0.14.0.zip'}
    lizard_firmware.selected_online_version = '0.14.0'
    await lizard_firmware.download()
    assert any('not supported' in message for message in notifications)
    assert lizard_firmware.local_version is None


async def test_flash_core_refuses_unsupported_local_version(lizard_firmware: LizardFirmware) -> None:
    """Flashing the core is refused when the local firmware version is not supported."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    lizard_firmware.local_version = '0.14.0'
    await lizard_firmware.flash_core()
    assert any('not supported' in message for message in notifications)


async def test_flash_core_refuses_unknown_local_version(lizard_firmware: LizardFirmware) -> None:
    """With a restriction in place, flashing the core is refused as long as the local version has not been read."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    await lizard_firmware.flash_core()
    assert any('version is unknown' in message for message in notifications)


async def test_flash_p0_refuses_unknown_core_version(lizard_firmware: LizardFirmware) -> None:
    """With a restriction in place, flashing the P0 is refused as long as the core version has not been read."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    await lizard_firmware.flash_p0()
    assert any('version is unknown' in message for message in notifications)


async def test_flash_core_with_supported_version_passes_version_check(lizard_firmware: LizardFirmware) -> None:
    """A supported local version passes the version check and proceeds to the readiness check."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    lizard_firmware.local_version = '0.13.0'
    await lizard_firmware.flash_core()
    assert not any('not supported' in message for message in notifications)
    assert any('not ready' in message for message in notifications)


async def test_flash_p0_refuses_unsupported_core_version(lizard_firmware: LizardFirmware) -> None:
    """Flashing the P0 is refused when the core firmware version is not supported."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    lizard_firmware.core_version = '0.14.0'
    await lizard_firmware.flash_p0()
    assert any('not supported' in message for message in notifications)


async def test_warning_when_core_runs_unsupported_version(lizard_firmware: LizardFirmware) -> None:
    """Reading an unsupported version from the Core warns that a supported version needs to be flashed."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    communication.core_version = 'v0.14.0'
    await connect(communication)
    task = background_tasks.create(lizard_firmware.read_core_version(), name='read core version')
    await forward(seconds=3.0)
    assert task.done()
    assert lizard_firmware.core_version == '0.14.0'
    assert any('Core is running Lizard 0.14.0' in message for message in notifications)


async def test_no_warning_when_core_runs_supported_version(lizard_firmware: LizardFirmware) -> None:
    """Reading a supported version from the Core does not warn."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    await connect(communication)
    task = background_tasks.create(lizard_firmware.read_core_version(), name='read core version')
    await forward(seconds=3.0)
    assert task.done()
    assert lizard_firmware.core_version == '0.13.0'
    assert not any('not supported' in message for message in notifications)


async def test_warning_when_p0_runs_unsupported_version(lizard_firmware: LizardFirmware) -> None:
    """Reading an unsupported version from the P0 warns that a supported version needs to be flashed."""
    notifications: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(notifications.append)
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    communication.p0_version = 'v0.14.0'
    await connect(communication)
    task = background_tasks.create(lizard_firmware.read_p0_version(), name='read p0 version')
    await forward(seconds=3.0)
    assert task.done()
    assert lizard_firmware.p0_version == '0.14.0'
    assert any('P0 is running Lizard 0.14.0' in message for message in notifications)
