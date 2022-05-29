from __future__ import annotations

import io
import os
from datetime import datetime
from glob import glob
from pathlib import Path

import humanize
import rosys
from PIL import Image, ImageDraw, ImageFont
from rosys.actors import Actor

from ..world import World

rosys_dir = os.path.dirname(os.path.dirname(__file__))
image_font = ImageFont.truetype(f'{rosys_dir}/RobotoMono-Medium.ttf', 12)


class TimelapsRecorder(Actor):
    interval: float = 1
    world: World
    storage_path: str = os.path.expanduser('~/.rosys/timelapse')

    def __init__(self) -> None:
        super().__init__()
        os.makedirs(self.storage_path, exist_ok=True)
        self.cover_font = ImageFont.truetype(f'{rosys_dir}/RobotoMono-Medium.ttf', 100)

    async def save(self, image: rosys.Image) -> None:
        await rosys.run.cpu_bound(save_image, image, self.storage_path)

    async def compress_video(self) -> None:
        jpgs = sorted(glob(f'{self.storage_path}/*.jpg'))
        if len(jpgs) < 2:
            return
        start = datetime.fromtimestamp(int(Path(jpgs[0]).stem))
        end = datetime.fromtimestamp(int(Path(jpgs[-1]).stem))
        self.log.info(f'creating video from {start} to {end}')
        duration = humanize.naturaldelta(end-start)
        for i in range(20):
            self.create_cover(start.strftime("%d.%m.%Y %H:%M:%S"), duration, f'{self.storage_path}/0_cover_{i}.jpg')
        id = start.strftime('%Y%m%d_%H-%M-%S_' + duration.replace(' ', '_'))
        target_dir = self.storage_path + '/' + id
        os.mkdir(target_dir)
        await rosys.run.sh(['mv', self.storage_path + '/*.jpg', target_dir])
        cmd = f'nice -n 19 ffmpeg -hide_banner -r 10 -pattern_type glob -i "{target_dir}/*.jpg" -s 1600x1200 -vcodec libx264 -crf 18 -preset slow -pix_fmt yuv420p -y {target_dir}/{id}.mp4; mv {target_dir}/*mp4 {self.storage_path}; rm -r {target_dir};'
        rosys.task_logger.create_task(rosys.run.sh(cmd, timeout=None))

    def clear_jpegs(self):
        files = glob(f'{self.storage_path}/**/*.jpg', recursive=True)
        for f in files:
            os.remove(f)

    def create_cover(self, title: str, subtilte: str, dest: str):
        img = Image.new('RGB', (1600, 1200), 'black')
        draw = ImageDraw.Draw(img)
        w, h = draw.textsize(title, font=self.cover_font)
        draw.text((img.width / 2 - w / 2, img.height / 2 - h / 2 - 200), title, font=self.cover_font, fill='white')
        w, h = draw.textsize(subtilte, font=self.cover_font)
        draw.text((img.width / 2 - w / 2, img.height / 2 - h / 2 + 100), subtilte, font=self.cover_font, fill='white')
        img.save(dest)

# static to run cpu bound


def save_image(image: rosys.world.Image, path: str):
    img = Image.open(io.BytesIO(image.data))
    draw = ImageDraw.Draw(img)
    x = img.width - 300
    y = img.height - 18
    text = f'{datetime.fromtimestamp(image.time).strftime("%Y-%m-%d %H:%M:%S")}, cam {image.camera_id}'
    # shadow
    draw.text((x - 1, y - 1), text, font=image_font, fill=(0, 0, 0))
    draw.text((x + 1, y - 1), text, font=image_font, fill=(0, 0, 0))
    draw.text((x - 1, y + 1), text, font=image_font, fill=(0, 0, 0))
    draw.text((x + 1, y + 1), text, font=image_font, fill=(0, 0, 0))
    draw.text((x, y), text, font=image_font, fill=(255, 255, 255))
    dest = path + f'/{int(image.time)}.jpg'
    img.save(dest, "JPEG")
