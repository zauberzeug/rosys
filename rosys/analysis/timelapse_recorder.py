from __future__ import annotations

import io
import logging
import os
from datetime import datetime
from glob import glob
from pathlib import Path
from typing import Optional, Protocol

import humanize
import rosys
from PIL import Image, ImageDraw, ImageFont

STORAGE_PATH = os.path.expanduser('~/.rosys/timelapse')
FONT = f'{os.path.dirname(__file__)}/assets/RobotoMono-Medium.ttf'
IMAGE_FONT = ImageFont.truetype(FONT, 12)
BIG_COVER_FONT = ImageFont.truetype(FONT, 100)
SMALL_COVER_FONT = ImageFont.truetype(FONT, 60)


class RosysImage(Protocol):
    camera_id: str
    time: float
    data: Optional[bytes] = None


class TimelapseRecorder:

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.timelapse_recorder')

        os.makedirs(STORAGE_PATH + '/videos', exist_ok=True)

    async def save(self, image: RosysImage) -> None:
        await rosys.run.cpu_bound(save_image, image, STORAGE_PATH)

    async def compress_video(self) -> None:
        jpgs = sorted(glob(f'{STORAGE_PATH}/*.jpg'))
        if len(jpgs) < 20:
            self.log.info(f'very few images ({len(jpgs)}); not creating video')
            return
        start_unix = int(Path(jpgs[0]).stem[:-4])
        start = datetime.fromtimestamp(start_unix)
        end = datetime.fromtimestamp(int(Path(jpgs[-1]).stem[:-4]))
        self.log.info(f'creating video from {start} to {end}')
        duration = humanize.naturaldelta(end-start)
        await self.create_info(start.strftime('%d.%m.%Y %H:%M:%S'), duration, time=start_unix - 1)
        id = start.strftime('%Y%m%d_%H-%M-%S_' + duration.replace(' ', '_'))
        target_dir = STORAGE_PATH + '/' + id
        os.makedirs(target_dir, exist_ok=True)
        os.makedirs(STORAGE_PATH + '/videos', exist_ok=True)
        self.log.info(await rosys.run.sh(f'mv {STORAGE_PATH}/*.jpg {target_dir}', shell=True))
        # it seems that niceness of subprocess is relative to own niceness, but we want an absolute niceness
        absolute_niceness = 10 - os.nice(0)
        cmd = f'nice -n {absolute_niceness} ffmpeg -hide_banner -threads 1 -r 10 -pattern_type glob -i "{target_dir}/*.jpg" -s 1600x1200 -vcodec libx264 -crf 18 -preset slow -pix_fmt yuv420p -y {target_dir}/{id}.mp4; mv {target_dir}/*mp4 {STORAGE_PATH}/videos; rm -r {target_dir};'
        self.log.info(f'starting {cmd}')
        rosys.task_logger.create_task(rosys.run.sh(cmd, timeout=None, shell=True), name='timelapse ffmpeg')

    def clear_jpegs(self) -> None:
        files = glob(f'{STORAGE_PATH}/**/*.jpg', recursive=True)
        for f in files:
            os.remove(f)

    async def create_info(self, title: str, subtitle: str, frames: int = 20, time: Optional[int] = None) -> None:
        prefix = f'{STORAGE_PATH}/{int(rosys.time() if time is None else time)}'
        await rosys.run.cpu_bound(save_info, title, subtitle, prefix, frames)


def save_image(image: RosysImage, path: str) -> None:
    img = Image.open(io.BytesIO(image.data))
    draw = ImageDraw.Draw(img)
    x = img.width - 300
    y = img.height - 18
    text = f'{datetime.fromtimestamp(image.time).strftime("%Y-%m-%d %H:%M:%S")}, cam {image.camera_id}'
    # shadow
    draw.text((x - 1, y - 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x + 1, y - 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x - 1, y + 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x + 1, y + 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x, y), text, font=IMAGE_FONT, fill=(255, 255, 255))
    filepath = path + f'/{int(image.time)}0000.jpg'
    img.save(filepath, 'JPEG')


def save_info(title: str, subtitle: str, prefix: str, frames: int) -> None:
    for i in range(frames):
        img = Image.new('RGB', (1600, 1200), 'black')
        draw = ImageDraw.Draw(img)
        font = BIG_COVER_FONT if len(subtitle) < 25 and len(title) < 25 else SMALL_COVER_FONT
        w, h = draw.textsize(title, font=font)
        draw.text((img.width / 2 - w / 2, img.height / 2 - h / 2 - 200), title, font=font, fill='white')
        w, h = draw.textsize(subtitle, font=font)
        draw.text((img.width / 2 - w / 2, img.height / 2 - h / 2 + 100), subtitle, font=font, fill='white')
        filepath = f'{prefix}{i:04}.jpg'
        img.save(filepath)
