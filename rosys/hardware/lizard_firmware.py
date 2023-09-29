import logging
import subprocess
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import requests

from .. import rosys
from ..run import awaitable
from .communication import SerialCommunication

if TYPE_CHECKING:
    from rosys.hardware.robot_brain import RobotBrain


class LizardFirmware:
    GITHUB_URL = 'https://api.github.com/repos/zauberzeug/lizard/releases/latest'
    PATH = Path('~/.lizard').expanduser()

    def __init__(self, robot_brain: 'RobotBrain') -> None:
        self.log = logging.getLogger('rosys.lizard_firmware')
        self.robot_brain = robot_brain

        self.flash_params: list[str] = []

        self.core_version: Optional[str] = None
        self.p0_version: Optional[str] = None
        self.local_version: Optional[str] = None
        self.online_version: Optional[str] = None

        self.local_checksum: Optional[str] = None
        self.core_checksum: Optional[str] = None

    async def read_all(self) -> None:
        await self.read_online_version()
        self.read_local_version()
        await self.read_core_version()
        await self.read_p0_version()
        self.read_local_checksum()
        await self.read_core_checksum()

    async def read_online_version(self) -> None:
        response: dict[str, str] = (await rosys.run.io_bound(requests.get, self.GITHUB_URL)).json()
        try:
            self.online_version = response['tag_name'].removeprefix('v')
        except KeyError:
            rosys.notify(response.get('message', 'Could not access online version'), 'warning')

    def read_local_version(self) -> None:
        path = self.PATH / 'build' / 'lizard.bin'
        with path.open('rb') as f:
            head = f.read(150).decode('utf-8', 'backslashreplace')
        self.local_version = head.split(' ')[3].replace('\x00', '').split('lizard')[0].removeprefix('v')

    async def read_core_version(self) -> None:
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('core.version()', 'version:', timeout=1):
                self.core_version = response.split()[-1].split('-')[0][1:]
                return
            self.log.warning('Could not read Lizard version from Core')

    async def read_p0_version(self) -> None:
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('p0.version()', 'p0:', timeout=1):
                self.p0_version = response.split()[-1].split('-')[0][1:]
                return
            self.log.warning('Could not read Lizard version from P0')

    def read_local_checksum(self) -> None:
        checksum = sum(ord(c) for c in self.robot_brain.lizard_code) % 0x10000
        self.local_checksum = f'{checksum:04x}'
        self.log.info(f'local checksum: {self.local_checksum}')

    async def read_core_checksum(self) -> None:
        deadline = rosys.time() + 5.0
        while rosys.time() < deadline:
            if response := await self.robot_brain.send_and_await('core.startup_checksum()', 'checksum:', timeout=1):
                self.core_checksum = response.split()[-1]
                self.log.info(f'core checksum: {self.core_checksum}')
                return
            self.log.warning('Could not read startup checksum from Core')

    @awaitable
    def download(self) -> None:
        url = requests.get(self.GITHUB_URL, timeout=5.0).json()['assets'][0]['browser_download_url']
        zip_path = self.PATH / 'lizard.zip'
        zip_path.write_bytes(requests.get(url, timeout=5.0).content)
        subprocess.run(['unzip', '-o', zip_path], cwd=self.PATH, check=True)
        zip_path.unlink()
        self.read_local_version()

    async def flash_core(self) -> None:
        assert isinstance(self.robot_brain.communication, SerialCommunication)
        rosys.notify(f'Flashing Lizard firmware {self.local_version} to Core...')
        self.robot_brain.communication.disconnect()
        await rosys.sleep(0.3)
        output = await rosys.run.sh(['./flash.py'] + self.flash_params, timeout=None, working_dir=self.PATH)
        self.log.info(f'flashed Lizard:\n {output}')
        self.robot_brain.communication.connect()
        await self.read_core_version()
        rosys.notify('Finished.', 'positive')

    async def flash_p0(self) -> None:
        rosys.notify(f'Flashing Lizard firmware {self.core_version} to P0...')
        await self.robot_brain.send('p0.flash()')
        start = rosys.time()
        while (self.robot_brain.hardware_time or 0) < start + 3.0:
            if rosys.time() > start + 60.0:
                rosys.notify('Failed.', 'negative')
                return
            await rosys.sleep(0.1)
        await self.robot_brain.restart()
        await rosys.sleep(3.0)
        await self.read_p0_version()
        rosys.notify('Finished.', 'positive')
