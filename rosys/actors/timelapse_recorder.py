from __future__ import annotations

import os
import shlex
import shutil
from asyncio import Task
from datetime import datetime
from glob import glob

import rosys
from rosys.actors import Actor
from world import World


class TimelapsRecorder(Actor):
    interval: float = 1
    world: World
    storage_path: str = os.path.expanduser('~/.rosys/timelapse')

    def __init__(self) -> None:
        super().__init__()
        os.makedirs(self.storage_path, exist_ok=True)
        # NOTE as long as we develop this feature we cleanup on start
        self.clear_jpegs()

    async def step(self):
        await super().step()

    async def save(self, image: rosys.Image) -> None:
        def _save(image: rosys.Image):
            with open(self.storage_path + f'/{image.time}.jpg', 'wb') as f:
                f.write(image.data)
        await rosys.run.io_bound(_save, image)

    async def compress_video(self) -> None:
        now = datetime.now().strftime('%Y-%m-%d_%H_%M_%S')
        target_dir = self.storage_path + '/' + now
        os.mkdir(target_dir)
        await rosys.run.sh(['mv', self.storage_path + '/*.jpg', target_dir])
        cmd = f'nice -n 19 ffmpeg -hide_banner -r 10 -pattern_type glob -i "{target_dir}/*.jpg" -s 1600x1200 -vcodec libx264 -crf 18 -preset slow -pix_fmt yuv420p -y {target_dir}/timelapse_{now}.mp4; mv {target_dir}/*mp4 {self.storage_path}; rm -r {target_dir};'
        rosys.task_logger.create_task(rosys.run.sh(cmd, timeout=None))

    def clear_jpegs(self):
        files = glob(f'{self.storage_path}/**/*.jpg', recursive=True)
        for f in files:
            os.remove(f)
