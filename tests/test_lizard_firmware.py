import asyncio
import re
from collections.abc import Generator
from pathlib import Path

import pytest
from nicegui import background_tasks
from packaging.specifiers import InvalidSpecifier, SpecifierSet
from test_robot_brain import CommunicationSimulation, connect  # pylint: disable=wrong-import-order

import rosys
from rosys.hardware import RobotBrain, RobotHardware
from rosys.hardware.lizard_firmware import FirmwareFlashError, LizardFirmware
from rosys.testing import forward


def app_image(version: str, project: str = 'lizard') -> bytes:
    """A minimal ESP32 app image header with an esp_app_desc_t at offset 0x20."""
    header = bytearray(112)
    header[32:36] = LizardFirmware.APP_DESC_MAGIC
    header[48:80] = version.encode().ljust(32, b'\x00')
    header[80:112] = project.encode().ljust(32, b'\x00')
    return bytes(header)


class VersionCommunicationSimulation(CommunicationSimulation):

    def __init__(self) -> None:
        super().__init__()
        self.core_version = 'v0.13.0'
        self.p0_version = 'v0.13.0'
        self.core_answers_pin_queries = False

    async def send(self, msg: str) -> None:
        await super().send(msg)
        line = msg.rsplit('@', 1)[0]
        if self.core_answers_pin_queries and (match := re.fullmatch(r'core\.get_pin_status\((\d+)\)', line)):
            self.incoming.append(f'GPIO_Status[{match.group(1)}]| Level: 0')
        if line == 'core.version()':
            self.incoming.append(f'version: {self.core_version}')
        if line == 'p0.version()':
            self.incoming.append(f'p0: version: {self.p0_version}')


@pytest.fixture
def lizard_firmware(rosys_integration: None) -> Generator[LizardFirmware, None, None]:
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


@pytest.mark.parametrize('target, attr, proceeds_to', [('core', 'local_version', 'does not answer'),
                                                       ('p0', 'core_version', 'not ready')])
@pytest.mark.parametrize('version, expected_message', [
    (None, 'version is unknown'),
    ('0.14.0', 'not supported'),
    ('0.13.0', None),
])
async def test_flash_version_gate(lizard_firmware: LizardFirmware, monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
                                  target: str, attr: str, proceeds_to: str,
                                  version: str | None, expected_message: str | None) -> None:
    """The version gate refuses unknown and unsupported versions; supported ones proceed to the next check."""
    expected_message = expected_message or proceeds_to
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    if version is not None:
        setattr(lizard_firmware, attr, version)
    task = background_tasks.create(getattr(lizard_firmware, f'flash_{target}')(), name=f'flash {target}')
    await forward(seconds=3.0)
    assert isinstance(task.exception(), FirmwareFlashError)
    assert expected_message in str(task.exception())


async def test_flash_core_reads_local_version(lizard_firmware: LizardFirmware,
                                              monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """Flashing the core reads the local version by itself if it has not been read yet."""
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    (tmp_path / 'build').mkdir()
    (tmp_path / 'build' / 'lizard.bin').write_bytes(app_image('v0.13.0'))
    task = background_tasks.create(lizard_firmware.flash_core(), name='flash core')
    await forward(seconds=3.0)
    assert lizard_firmware.local_version == '0.13.0'
    assert 'does not answer' in str(task.exception())


async def test_flash_core_reads_version_of_unready_core(lizard_firmware: LizardFirmware,
                                                        monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """Flashing a Core that never got ready leaves its version known, so flashing the P0 is not refused for it."""
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    communication.core_answers_pin_queries = True
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    monkeypatch.setattr('rosys.hardware.lizard_firmware.SerialCommunication', VersionCommunicationSimulation)
    monkeypatch.setattr('rosys.hardware.lizard_firmware.SUDO_UNBUFFERED', [])
    (tmp_path / 'build').mkdir()
    (tmp_path / 'build' / 'lizard.bin').write_bytes(app_image('v0.13.0'))
    (tmp_path / 'flash.py').write_text('#!/bin/sh\n')
    (tmp_path / 'flash.py').chmod(0o755)
    task = background_tasks.create(lizard_firmware.flash_core(), name='flash core')
    await forward(until=task.done, timeout=1000)
    assert task.exception() is None
    assert not lizard_firmware.robot_brain.is_ready
    assert lizard_firmware.core_version == '0.13.0'
    with pytest.raises(FirmwareFlashError, match='not ready'):
        await lizard_firmware.flash_p0()


async def test_flash_p0_fails_when_p0_does_not_answer(lizard_firmware: LizardFirmware) -> None:
    """A P0 that does not answer the strapping pin query fails the flash with a ``FirmwareFlashError``."""
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    await connect(communication)
    lizard_firmware.core_version = '0.13.0'
    task = background_tasks.create(lizard_firmware.flash_p0(), name='flash p0')
    await forward(seconds=3.0)
    assert isinstance(task.exception(), FirmwareFlashError)
    assert str(task.exception()) == 'Flashing P0 failed. P0 does not answer.'
    assert lizard_firmware.progress == 'Flashing P0 failed. P0 does not answer.'


async def test_stream_reports_progress(lizard_firmware: LizardFirmware) -> None:
    """The last output line becomes the progress."""
    await lizard_firmware._stream(['sh', '-c', 'echo one; echo two'])  # pylint: disable=protected-access
    assert lizard_firmware.progress == 'two'


async def test_stream_raises_on_non_zero_exit(lizard_firmware: LizardFirmware) -> None:
    """A non-zero exit raises with the last output line."""
    with pytest.raises(FirmwareFlashError, match='two'):
        await lizard_firmware._stream(['sh', '-c', 'echo one; echo two; exit 1'])  # pylint: disable=protected-access
    assert lizard_firmware.progress == 'Flashing failed: two'


async def test_stream_strips_colour_codes(lizard_firmware: LizardFirmware) -> None:
    """ANSI colour codes of the script output reach neither the progress nor the error message."""
    script = r"printf '\033[1m\033[91mfailed\033[0m\033[0m\n'; exit 1"
    with pytest.raises(FirmwareFlashError, match=r'^Flashing failed: failed$'):
        await lizard_firmware._stream(['sh', '-c', script])  # pylint: disable=protected-access
    assert lizard_firmware.progress == 'Flashing failed: failed'


async def test_stream_terminates_script_on_cancellation(lizard_firmware: LizardFirmware, tmp_path: Path) -> None:
    """Cancelling the stream ends the script before the cancellation propagates."""
    marker = tmp_path / 'marker'
    script = f'echo started; sleep 0.3; touch {marker}'
    task = asyncio.create_task(lizard_firmware._stream(['sh', '-c', script]))  # pylint: disable=protected-access
    while lizard_firmware.progress != 'started':
        await asyncio.sleep(0.01)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    await asyncio.sleep(0.6)
    assert not marker.exists()


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


@pytest.mark.parametrize('target', ['core', 'p0'])
@pytest.mark.parametrize('reported, project, version', [
    ('v0.13.0', 'lizard', '0.13.0'),                              # release build before project names
    ('v0.13.0-3-g248f4ed-dirty', 'lizard', '0.13.0'),             # dev build before project names
    ('lizard v0.13.0', 'lizard', '0.13.0'),                       # release build with project name
    ('lizard-zz v0.0.2', 'lizard-zz', '0.0.2'),                   # integrator release build
    ('lizard-zz v0.0.1-6-g69dfddf-dirty', 'lizard-zz', '0.0.1'),  # integrator dev build
    ('unknown', None, None),                                      # built without git
])
async def test_read_version(lizard_firmware: LizardFirmware,
                            target: str, reported: str, project: str | None, version: str | None) -> None:
    """Version responses with and without a project name are parsed into project and release version."""
    communication = lizard_firmware.robot_brain.communication
    assert isinstance(communication, VersionCommunicationSimulation)
    setattr(communication, f'{target}_version', reported)
    await connect(communication)
    read_version = getattr(lizard_firmware, f'read_{target}_version')
    task = background_tasks.create(read_version(), name=f'read {target} version')
    await forward(seconds=3.0)
    assert task.done()
    assert getattr(lizard_firmware, f'{target}_version') == version
    assert getattr(lizard_firmware, f'{target}_project') == project


@pytest.mark.parametrize('files, project, version', [
    ({'lizard.bin': app_image('v0.13.0')}, 'lizard', '0.13.0'),
    ({'lizard-zz.bin': app_image('v0.0.1-6-g69dfddf', 'lizard-zz')}, 'lizard-zz', '0.0.1'),
    ({'lizard.bin': b'not an app image'}, None, None),
    ({}, None, None),
    ({'bootloader.bin': b'\xff' * 8192, 'lizard.bin': app_image('v0.13.0')}, 'lizard', '0.13.0'),
    ({'lizard-zz.bin': app_image('v0.0.1', 'lizard-zz'), 'lizard.bin': app_image('v0.13.0')}, 'lizard', '0.13.0'),
])
async def test_read_local_version(lizard_firmware: LizardFirmware, monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
                                  files: dict[str, bytes], project: str | None, version: str | None) -> None:
    """The local project and version are read from the esp_app_desc_t header of the app image in the build directory.

    Other binaries of an ESP-IDF build are skipped by their missing magic word, ``lizard.bin`` (the file ``flash.py``
    flashes) takes precedence over other app images, and a previously read version is reset when nothing is found.
    """
    monkeypatch.setattr(LizardFirmware, 'PATH', tmp_path)
    (tmp_path / 'build').mkdir()
    for filename, content in files.items():
        (tmp_path / 'build' / filename).write_bytes(content)
    lizard_firmware.local_version = '0.12.0'
    lizard_firmware.local_project = 'lizard'
    lizard_firmware.read_local_version()
    assert lizard_firmware.local_version == version
    assert lizard_firmware.local_project == project
