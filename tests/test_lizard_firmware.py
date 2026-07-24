from collections.abc import AsyncGenerator
from pathlib import Path

import pytest
from nicegui import background_tasks
from packaging.specifiers import InvalidSpecifier, SpecifierSet
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
async def lizard_firmware(rosys_integration: None) -> AsyncGenerator[LizardFirmware, None]:
    """A firmware restricted to Lizard versions below 0.14.0."""
    robot_brain = RobotBrain(VersionCommunicationSimulation(),
                             enable_esp_on_startup=False,
                             supported_lizard_versions='<0.14.0')
    _robot = RobotHardware([], robot_brain)  # NOTE: keep alive so its weak update repeater keeps running
    yield robot_brain.lizard_firmware


@pytest.fixture
def notifications(rosys_integration: None) -> list[str]:
    """All notification messages emitted during the test."""
    messages: list[str] = []
    rosys.NEW_NOTIFICATION.subscribe(messages.append)
    return messages


@pytest.mark.parametrize('specifier, version, expected', [
    (None, '0.12.0', True),
    (None, '0.14.0', True),
    (None, '99.0.0', True),
    ('<0.14.0', '0.13.0', True),
    ('<0.14.0', '0.14.0', False),
    ('>0.12.0,<0.14.0', '0.12.0', False),
    ('>0.12.0,<0.14.0', '0.13.9', True),
    ('<0.14.0', '', False),
    ('<0.14.0', 'not-a-version', False),
])
def test_is_version_supported(lizard_firmware: LizardFirmware,
                              specifier: str | None, version: str, expected: bool) -> None:
    """Without a specifier all versions are supported; with one, violating and unparsable versions are not."""
    lizard_firmware.supported_versions = SpecifierSet(specifier) if specifier else None
    assert lizard_firmware.is_version_supported(version) == expected


async def test_invalid_specifier(rosys_integration: None) -> None:
    """An invalid specifier fails fast when constructing the robot brain."""
    with pytest.raises(InvalidSpecifier):
        RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False, supported_lizard_versions='latest')


async def test_online_versions(lizard_firmware: LizardFirmware, monkeypatch: pytest.MonkeyPatch) -> None:
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


async def test_download_refusal(lizard_firmware: LizardFirmware, notifications: list[str]) -> None:
    """Downloading an unsupported version is refused even if it was selected programmatically."""
    lizard_firmware.online_versions = {'0.14.0': 'https://example.com/lizard-0.14.0.zip'}
    lizard_firmware.selected_online_version = '0.14.0'
    await lizard_firmware.download()
    assert any('not supported' in message for message in notifications)
    assert lizard_firmware.local_version is None


@pytest.mark.parametrize('target, attr', [('core', 'local_version'), ('p0', 'core_version')])
@pytest.mark.parametrize('version, expected_message', [
    (None, 'version is unknown'),
    ('0.14.0', 'not supported'),
    ('0.13.0', 'not ready'),
])
async def test_flash_version_gate(lizard_firmware: LizardFirmware, notifications: list[str],
                                  monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
                                  target: str, attr: str, version: str | None, expected_message: str) -> None:
    """The version gate refuses unknown and unsupported versions; supported ones proceed to the readiness check."""
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    if version is not None:
        setattr(lizard_firmware, attr, version)
    await getattr(lizard_firmware, f'flash_{target}')()
    assert any(expected_message in message for message in notifications)


async def test_flash_core_reads_local_version(lizard_firmware: LizardFirmware, notifications: list[str],
                                              monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """Flashing the core reads the local version by itself if it has not been read yet."""
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    (tmp_path / 'build').mkdir()
    (tmp_path / 'build' / 'lizard.bin').write_bytes(b'v0.13.0\x00lizard')
    await lizard_firmware.flash_core()
    assert lizard_firmware.local_version == '0.13.0'
    assert not any('version is unknown' in message for message in notifications)
    assert any('not ready' in message for message in notifications)


@pytest.mark.parametrize('target', ['core', 'p0'])
@pytest.mark.parametrize('version, expect_warning', [('v0.13.0', False), ('v0.14.0', True)])
async def test_version_warning(lizard_firmware: LizardFirmware, notifications: list[str],
                               target: str, version: str, expect_warning: bool) -> None:
    """Reading an unsupported version from the microcontroller warns, reading a supported one does not."""
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    setattr(communication, f'{target}_version', version)
    await connect(communication)
    read_version = getattr(lizard_firmware, f'read_{target}_version')
    task = background_tasks.create(read_version(), name=f'read {target} version')
    await forward(seconds=3.0)
    assert task.done()
    assert getattr(lizard_firmware, f'{target}_version') == version.removeprefix('v')
    assert any('not supported' in message for message in notifications) == expect_warning
