import logging
import re
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
    VERSION_LINE_PATTERN = re.compile(r'version: (?:(?P<project>\S+) )?(?P<version>\S+)')
    VERSION_PATTERN = re.compile(r'v?(\d+\.\d+\.\d+)')
    APP_DESC_MAGIC = b'\x32\x54\xcd\xab'  # marks the esp_app_desc_t at offset 0x20 of an ESP32 app image

    def __init__(self, robot_brain: 'RobotBrain', *, supported_versions: str | None = None) -> None:
        """
        :param robot_brain: the Robot Brain instance to use for reading and flashing Lizard firmware
        :param supported_versions: PEP 440 version specifier restricting which Lizard versions can be
            downloaded and flashed, e.g. ``'<0.14.0'`` (default: ``None``, all versions are supported)
        :raises InvalidSpecifier: When ``supported_versions`` is not a valid version specifier
        """
        self.log = logging.getLogger('rosys.lizard_firmware')
        self.robot_brain = robot_brain
        self.supported_versions = SpecifierSet(supported_versions) if supported_versions else None

        self.flash_params: list[str] = []

        self.core_version: str | None = None
        self.core_project: str | None = None
        self.p0_version: str | None = None
        self.p0_project: str | None = None
        self.local_version: str | None = None
        self.local_project: str | None = None
        self.online_versions: dict[str, str] = {}
        self.selected_online_version: str | None = None

        self.local_checksum: str | None = None
        self.core_checksum: str | None = None

        self._p0_flash_last_complete: float = 0.0
        self.robot_brain.FLASH_P0_COMPLETE.subscribe(lambda: setattr(self, '_p0_flash_last_complete', rosys.time()))

    @property
    def core_description(self) -> str:
        """Project name and version of the firmware on the Core, e.g. ``'lizard-zz 0.0.2'``."""
        return ' '.join(filter(None, (self.core_project, self.core_version))) or '?'

    @property
    def p0_description(self) -> str:
        """Project name and version of the firmware on the P0."""
        return ' '.join(filter(None, (self.p0_project, self.p0_version))) or '?'

    @property
    def local_description(self) -> str:
        """Project name and version of the downloaded firmware binary."""
        return ' '.join(filter(None, (self.local_project, self.local_version))) or '?'

    @property
    def checksums_match(self) -> bool | None:
        """Whether the local and core startup checksums match, or ``None`` if either checksum is unknown."""
        if self.local_checksum is None or self.core_checksum is None:
            return None
        return self.local_checksum == self.core_checksum

    def is_version_supported(self, version: str) -> bool:
        """Whether the given Lizard ``version`` satisfies the ``supported_versions`` specifier."""
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
                    self.log.debug('Lizard %s is not supported by this Robot Brain. Skipping.', version_name)
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
        path = next((self.PATH / 'build').glob('*.bin'), self.PATH / 'build' / 'lizard.bin')
        with path.open('rb') as f:
            header = f.read(112)  # image header plus esp_app_desc_t up to the project name
        if header[32:36] != self.APP_DESC_MAGIC:
            self.log.error('%s is not an ESP32 app image', path)
            return
        self.local_version = self._parse_version(header[48:80].split(b'\x00', 1)[0].decode('utf-8', 'replace'))
        self.local_project = header[80:112].split(b'\x00', 1)[0].decode('utf-8', 'replace') or None

    async def read_core_version(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Could not read Lizard version from Core. Robot Brain is not ready.')
            return
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('core.version()', 'version:', timeout=1):
                self.core_project, self.core_version = self._parse_version_line(response)
                self._refuse_version(self.core_version, 'Core')
                return
        self.log.error('Could not read Lizard version from Core')

    async def read_p0_version(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Could not read Lizard version from P0. Robot Brain is not ready.')
            return
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('p0.version()', 'p0:', timeout=1):
                self.p0_project, self.p0_version = self._parse_version_line(response)
                self._refuse_version(self.p0_version, 'P0')
                return
        self.log.error('Could not read Lizard version from P0')

    @classmethod
    def _parse_version_line(cls, response: str) -> tuple[str | None, str | None]:
        """Parse a ``version: [<project>] <version>`` response into project name and version.

        Firmware built before the project name was reported yields ``'lizard'``,
        an unparsable response ``(None, None)``.
        """
        match = cls.VERSION_LINE_PATTERN.search(response)
        version = cls._parse_version(match.group('version')) if match else None
        if match is None or version is None:
            return None, None
        return match.group('project') or 'lizard', version

    @classmethod
    def _parse_version(cls, token: str) -> str | None:
        """Reduce a version token like ``'v0.13.0-3-g248f4ed-dirty'`` to its release version, e.g. ``'0.13.0'``."""
        match = cls.VERSION_PATTERN.match(token)
        return match.group(1) if match else None

    def read_local_checksum(self) -> None:
        checksum = sum(self.robot_brain.lizard_code.encode()) % 0x10000
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
        if self._refuse_version(self.selected_online_version, 'Downloading failed'):
            return
        assert self.selected_online_version in self.online_versions
        url = self.online_versions[self.selected_online_version]
        zip_path = self.PATH / 'lizard.zip'
        zip_path.write_bytes(requests.get(url, timeout=5.0).content)
        subprocess.run(['unzip', '-o', zip_path], cwd=self.PATH, check=True)
        zip_path.unlink()
        self.read_local_version()

    async def flash_core(self) -> None:
        if self.local_version is None:
            try:
                self.read_local_version()
            except FileNotFoundError:
                pass
        if self._refuse_version(self.local_version, 'Flashing Core failed'):
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
        if self._refuse_version(self.core_version, 'Flashing P0 failed'):
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

    def _refuse_version(self, version: str | None, context: str) -> bool:
        """Notify when the given version is unknown or violates the ``supported_versions`` specifier.

        :param version: the Lizard version to check (``None`` if unknown)
        :param context: prefix for the notification, e.g. ``'Core'`` or ``'Flashing Core failed'``
        :return: ``True`` if a restriction is configured and the version is unknown or not supported
        """
        if self.supported_versions is None:
            return False
        if version is None:
            rosys.notify(f'{context}: The Lizard version is unknown.', 'negative')
            return True
        if self.is_version_supported(version):
            return False
        msg = f'{context}: Lizard {version} is not supported by this Robot Brain (requires {self.supported_versions}).'
        rosys.notify(msg, 'negative', log_level=logging.WARNING)
        return True
