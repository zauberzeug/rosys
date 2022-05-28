from __future__ import annotations

import io
import os
import shlex
import shutil
from asyncio import Task
from datetime import datetime
from glob import glob

import rosys
from PIL import Image, ImageDraw, ImageFont, ImageStat
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
        rosys_dir = os.path.dirname(os.path.dirname(__file__))
        self.font = ImageFont.truetype(f'{rosys_dir}/RobotoMono-Medium.ttf', 12)

    async def step(self):
        await super().step()

    async def save(self, image: rosys.Image) -> None:
        def _save(image: rosys.world.Image):
            img = Image.open(io.BytesIO(image.data))
            draw = ImageDraw.Draw(img)
            x = img.width - 300
            y = img.height - 18
            text = f'{datetime.utcfromtimestamp(image.time).strftime("%Y-%m-%d %H:%M:%S")}, cam {image.camera_id}'
            # shadow
            draw.text((x - 1, y - 1), text, font=self.font, fill=(0, 0, 0))
            draw.text((x + 1, y - 1), text, font=self.font, fill=(0, 0, 0))
            draw.text((x - 1, y + 1), text, font=self.font, fill=(0, 0, 0))
            draw.text((x + 1, y + 1), text, font=self.font, fill=(0, 0, 0))
            draw.text((x, y), text, font=self.font, fill=(255, 255, 255))
            dest = self.storage_path + f'/{image.time}.jpg'
            img.save(dest, "JPEG")
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
