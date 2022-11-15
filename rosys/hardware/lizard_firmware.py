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

        self.active_version: Optional[str] = None
        self.offline_version: Optional[str] = None
        self.online_version: Optional[str] = None

        rosys.on_repeat(self.read_all, 60.0)

    async def read_all(self) -> None:
        await self.read_online_version()
        self.read_offline_version()
        await self.read_active_version()

    @awaitable
    def read_online_version(self) -> None:
        self.online_version = requests.get(self.GITHUB_URL).json()['tag_name'].removeprefix('v')

    def read_offline_version(self) -> None:
        bin = self.PATH / 'build' / 'lizard.bin'
        with open(bin, 'rb') as f:
            head = f.read(150).decode('utf-8', 'backslashreplace')
        self.offline_version = head.split(' ')[3].replace('\x00', '').split('lizard')[0].removeprefix('v')

    async def read_active_version(self, timeout: float = 5.0) -> None:
        deadline = rosys.time() + timeout
        while rosys.time() < deadline:
            response = await self.robot_brain.send_and_await('core.info()', 'lizard', timeout=1)
            if response:
                self.active_version = response.split()[-1].split('-')[0][1:]
                return
            self.log.warning('Could not get Lizard version')
            await rosys.sleep(0.1)

    @awaitable
    def download(self) -> None:
        url = requests.get(self.GITHUB_URL).json()['assets'][0]['browser_download_url']
        zip = self.PATH / 'lizard.zip'
        zip.write_bytes(requests.get(url).content)
        subprocess.run(['unzip', '-o', zip], cwd=self.PATH)
        zip.unlink()

    async def flash(self) -> None:
        assert isinstance(self.robot_brain.communication, SerialCommunication)
        rosys.notify(f'Flashing Lizard firmware {self.offline_version}...')
        self.robot_brain.communication.disconnect()
        await rosys.sleep(0.3)
        output = await rosys.run.sh(['./flash.py'] + self.flash_params, timeout=None, working_dir=self.PATH)
        self.log.info(f'flashed Lizard:\n {output}')
        self.robot_brain.communication.connect()
        rosys.notify(f'Finished.')
