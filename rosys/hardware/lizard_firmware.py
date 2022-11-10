import logging
import os
import pathlib
import subprocess
from typing import TYPE_CHECKING, Optional

import requests

import rosys

from ..run import awaitable
from .communication import SerialCommunication

if TYPE_CHECKING:
    from rosys.hardware.robot_brain import RobotBrain


class LizardFirmware:

    def __init__(self, robot_brain: 'RobotBrain') -> None:
        self.log = logging.getLogger('rosys.lizard_firmware')
        self.robot_brain = robot_brain
        self.lizard_version: Optional[str] = None
        self.latest_lizard_release: Optional[str] = None
        self.flash_params: list[str] = []

    async def flash(self) -> None:
        assert isinstance(self.robot_brain.communication, SerialCommunication)
        rosys.notify(f'Installing Lizard firmware {self.available_lizard_version}')
        self.robot_brain.communication.disconnect()
        output = await rosys.run.sh(['./flash.py'] + self.flash_params, timeout=None, working_dir=os.path.expanduser('~/.lizard'))
        self.log.info(f'flashed Lizard:\n {output}')
        self.robot_brain.communication.connect()
        await self.robot_brain.configure()
        await self.determine_lizard_version()
        rosys.notify(f'Installed Lizard firmware {self.lizard_version}')

    async def ensure_lizard_version(self) -> None:
        await self.determine_lizard_version()
        if self.lizard_version != self.available_lizard_version:
            await self.flash()

    async def determine_lizard_version(self) -> str:
        while (response := await self.robot_brain.send_and_await('core.info()', 'lizard', timeout=1)) is None:
            self.log.warning('Could not get Lizard version')
            await rosys.sleep(0.1)
            continue
        self.lizard_version = response.split()[-1].split('-')[0][1:]
        self.log.info(f'currently installed Lizard version is {self.lizard_version}')
        self.latest_lizard_release = await self.get_latest_lizard_release()

    async def update_lizard(self) -> None:
        rosys.notify(f'downloading Lizard {self.latest_lizard_release}')
        await self.download_latest_lizard_release()
        await self.flash()

    @awaitable
    def get_latest_lizard_release(self) -> str:
        return requests.get('https://api.github.com/repos/zauberzeug/lizard/releases/latest').json()['tag_name'].replace('v', '')

    @awaitable
    def download_latest_lizard_release(self) -> None:
        latest = 'https://api.github.com/repos/zauberzeug/lizard/releases/latest'
        zip = requests.get(latest).json()['assets'][0]['browser_download_url']
        with open(os.path.expanduser('~/.lizard/lizard.zip'), 'wb') as f:
            f.write(requests.get(zip).content)
        subprocess.run(['unzip', '-o', 'lizard.zip'], cwd=os.path.expanduser('~/.lizard'))
        pathlib.Path('~/.lizard/lizard.zip').expanduser().unlink()

    @property
    def available_lizard_version(self) -> str:
        with open(os.path.expanduser('~/.lizard/build/lizard.bin'), 'rb') as f:
            head = f.read(150).decode('utf-8', 'backslashreplace')
            version = head.split(' ')[3].replace('\x00', '')
            return version[1:version.find('lizard')].strip()

    @property
    def can_update(self) -> bool:
        return self.available_lizard_version != self.latest_lizard_release
