import logging
import subprocess
from pathlib import Path
from typing import TYPE_CHECKING

import requests
from packaging.specifiers import SpecifierSet
from packaging.version import InvalidVersion, Version

from .. import rosys
from ..run import awaitable
from .communication import SerialCommunication

if TYPE_CHECKING:
    from rosys.hardware.robot_brain import RobotBrain


class LizardFirmware:
    GITHUB_URL = 'https://api.github.com/repos/zauberzeug/lizard/releases'
    PATH = Path('~/.lizard').expanduser()
    PATH.mkdir(exist_ok=True)

    def __init__(self, robot_brain: 'RobotBrain', *, supported_versions: str | None = None) -> None:
        """
        :param supported_versions: PEP 440 version specifier restricting which Lizard versions can be
            downloaded and flashed, e.g. ``'<0.14.0'`` (default: ``None``, all versions are supported)
        :raises InvalidSpecifier: When ``supported_versions`` is not a valid version specifier
        """
        self.log = logging.getLogger('rosys.lizard_firmware')
        self.robot_brain = robot_brain
        self.supported_versions = SpecifierSet(supported_versions) if supported_versions else None

        self.flash_params: list[str] = []

        self.core_version: str | None = None
        self.p0_version: str | None = None
        self.local_version: str | None = None
        self.online_versions: dict[str, str] = {}
        self.selected_online_version: str | None = None

        self.local_checksum: str | None = None
        self.core_checksum: str | None = None

        self._p0_flash_last_complete: float = 0.0
        self.robot_brain.FLASH_P0_COMPLETE.subscribe(lambda: setattr(self, '_p0_flash_last_complete', rosys.time()))

    @property
    def checksums_match(self) -> bool | None:
        """Whether the local and core startup checksums match, or ``None`` if either checksum is unknown."""
        if self.local_checksum is None or self.core_checksum is None:
            return None
        return self.local_checksum == self.core_checksum

    def is_version_supported(self, version: str) -> bool:
        """Whether the given Lizard version satisfies the ``supported_versions`` specifier.

        :param version: the Lizard version to check
        :return: ``True`` if no specifier is configured or the version satisfies it,
            ``False`` for unparsable versions
        """
        if self.supported_versions is None:
            return True
        try:
            return self.supported_versions.contains(Version(version))
        except InvalidVersion:
            return False

    async def read_all(self) -> None:
        await self.read_online_version()
        self.read_local_version()
        await self.read_core_version()
        await self.read_p0_version()
        self.read_local_checksum()
        await self.read_core_checksum()

    async def read_online_version(self) -> None:
        response = await rosys.run.io_bound(requests.get, self.GITHUB_URL)
        if response is None:
            return
        response_data = response.json()
        for item in response_data:
            try:
                assert 'tag_name' in item
                version_name = item['tag_name'].removeprefix('v')
                if not self.is_version_supported(version_name):
                    self.log.debug('Skipping Lizard version %s which is not supported by this Robot Brain',
                                   version_name)
                    continue
                assert 'assets' in item
                browser_download_url = item['assets'][0]['browser_download_url']
                if not browser_download_url.endswith('.zip'):
                    continue
                self.online_versions[version_name] = browser_download_url
            except KeyError:
                rosys.notify(response_data.get('message', 'Could not access online version'), 'warning')
                break

    def read_local_version(self) -> None:
        path = self.PATH / 'build' / 'lizard.bin'
        with path.open('rb') as f:
            head = f.read(150).decode('utf-8', 'backslashreplace')
        self.local_version = head.replace('\x00', '').split('lizard')[0].split('v')[-1]

    async def read_core_version(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Could not read Lizard version from Core. Robot Brain is not ready.')
            return
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('core.version()', 'version:', timeout=1):
                self.core_version = response.split()[-1].split('-')[0][1:]
                self._notify_unsupported_version(self.core_version, 'Core')
                return
        self.log.error('Could not read Lizard version from Core')

    async def read_p0_version(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Could not read Lizard version from P0. Robot Brain is not ready.')
            return
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('p0.version()', 'p0:', timeout=1):
                self.p0_version = response.split()[-1].split('-')[0][1:]
                self._notify_unsupported_version(self.p0_version, 'P0')
                return
        self.log.error('Could not read Lizard version from P0')

    def read_local_checksum(self) -> None:
        checksum = sum(ord(c) for c in self.robot_brain.lizard_code) % 0x10000
        self.local_checksum = f'{checksum:04x}'
        self.log.info('local checksum: %s', self.local_checksum)

    async def read_core_checksum(self) -> None:
        self.core_checksum = None
        if not self.robot_brain.is_ready:
            self.log.error('Could not read startup checksum from Core. Robot Brain is not ready.')
            return
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline and self.robot_brain.is_ready:
            if response := await self.robot_brain.send_and_await('core.startup_checksum()', 'checksum:', timeout=1):
                self.core_checksum = response.split()[-1]
                self.log.info('core checksum: %s', self.core_checksum)
                return
        self.log.error('Could not read startup checksum from Core')

    @awaitable
    def download(self) -> None:
        if not self.selected_online_version:
            rosys.notify('No version selected.', 'warning')
            return
        if self._notify_unsupported_version(self.selected_online_version, 'Downloading failed'):
            return
        assert self.selected_online_version in self.online_versions
        url = self.online_versions[self.selected_online_version]
        zip_path = self.PATH / 'lizard.zip'
        zip_path.write_bytes(requests.get(url, timeout=5.0).content)
        subprocess.run(['unzip', '-o', zip_path], cwd=self.PATH, check=True)
        zip_path.unlink()
        self.read_local_version()

    async def flash_core(self) -> None:
        if self.supported_versions is not None:
            if self.local_version is None:
                try:
                    self.read_local_version()
                except FileNotFoundError:
                    pass
            if self.local_version is None:
                rosys.notify('Flashing Core failed. The local Lizard version is unknown.', 'negative')
                return
            if self._notify_unsupported_version(self.local_version, 'Flashing Core failed'):
                return
        if not self.robot_brain.is_ready:
            rosys.notify('Flashing Core failed. Robot Brain is not ready.', 'negative')
            return
        assert isinstance(self.robot_brain.communication, SerialCommunication)
        rosys.notify(f'Flashing Lizard firmware {self.local_version} to Core...')
        # NOTE: GPIO0 idles high in normal operation and is pulled low by Lizard itself during flashing,
        # so only GPIO2 and GPIO12 can be pre-checked here (https://github.com/zauberzeug/rosys/issues/430)
        if any(await self.robot_brain.esp_pins_core.get_strapping_pins(numbers=(2, 12))):
            rosys.notify('Flashing Core failed. Check strapping pins.', 'negative')
            return
        self.robot_brain.communication.disconnect()
        await rosys.sleep(0.3)
        output = await rosys.run.sh(['sudo', './flash.py', *self.flash_params], timeout=None, working_dir=self.PATH)
        self.log.info('flashed Lizard:\n%s', output)
        self.robot_brain.communication.connect()
        await self.read_core_version()
        rosys.notify('Finished.', 'positive')

    async def flash_p0(self, timeout: float = 120) -> None:
        if self.supported_versions is not None:
            if self.core_version is None:
                rosys.notify('Flashing P0 failed. The Core Lizard version is unknown.', 'negative')
                return
            if self._notify_unsupported_version(self.core_version, 'Flashing P0 failed'):
                return
        if not self.robot_brain.is_ready:
            rosys.notify('Flashing P0 failed. Robot Brain is not ready.', 'negative')
            return
        rosys.notify(f'Flashing Lizard firmware {self.core_version} to P0...')
        # NOTE: GPIO0 idles high in normal operation and is pulled low by Lizard itself during flashing,
        # so only GPIO2 and GPIO12 can be pre-checked here (https://github.com/zauberzeug/rosys/issues/430)
        if any(await self.robot_brain.esp_pins_p0.get_strapping_pins(numbers=(2, 12))):
            rosys.notify('Flashing P0 failed. Check strapping pins.', 'negative')
            return
        await self.robot_brain.send('p0.flash()')
        start = rosys.time()
        deadline = start + timeout
        while self._p0_flash_last_complete < start:
            if rosys.time() > deadline:
                rosys.notify('Failed.', 'negative')
                return
            await rosys.sleep(0.1)
        await self.robot_brain.restart()
        await rosys.sleep(3.0)
        await self.read_p0_version()
        rosys.notify('Finished.', 'positive')

    def _notify_unsupported_version(self, version: str, context: str) -> bool:
        """Notify when the given version violates the ``supported_versions`` specifier.

        :param version: the Lizard version to check
        :param context: prefix for the notification, e.g. ``'Core'`` or ``'Flashing Core failed'``
        :return: ``True`` if the version is not supported and a notification was sent
        """
        if self.is_version_supported(version):
            return False
        rosys.notify(f'{context}: Lizard {version} is not supported by this Robot Brain '
                     f'(requires {self.supported_versions}).', 'negative', log_level=logging.WARNING)
        return True
