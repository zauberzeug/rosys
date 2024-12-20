from __future__ import annotations

import io
import logging
import os
from asyncio import Task
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from typing import Protocol

import humanize
from cairosvg import svg2png
from PIL import Image, ImageDraw, ImageFont

from .. import rosys
from ..vision import Camera, ImageSize

STORAGE_PATH = Path('~/.rosys/timelapse').expanduser()
VIDEO_PATH = STORAGE_PATH / 'videos'
FONT = str(Path(__file__).parent / 'assets' / 'RobotoMono-Medium.ttf')
IMAGE_FONT = ImageFont.truetype(FONT, 22)
BIG_COVER_FONT = ImageFont.truetype(FONT, 50)
SMALL_COVER_FONT = ImageFont.truetype(FONT, 30)


class RosysImage(Protocol):
    camera_id: str
    time: float
    data: bytes | None
    size: ImageSize


class TimelapseRecorder:

    def __init__(self, *, width: int = 800, height: int = 600, capture_rate: float = 1) -> None:
        """Creates a time lapse recorder to capture images from a camera and creates a video of the sequence afterwards.

        Start the capturing by setting the ``camera`` property to a ``Camera`` instance.
        Setting to ``None`` stops capturing.
        After capturing, call ``compress_video`` to create a video from the captured images.

        :param width: width of the images to capture (default: 800)
        :param height: height of the images to capture (default: 600)
        :param capture_rate: images per second to capture (default: 1.0)
        """
        STORAGE_PATH.mkdir(parents=True, exist_ok=True)
        VIDEO_PATH.mkdir(parents=True, exist_ok=True)

        self.log = logging.getLogger('rosys.timelapse_recorder')
        self.width = width
        self.height = height
        self.avoid_broken_images = True
        self.capture_rate = capture_rate
        self.last_capture_time = rosys.time()
        self._notifications: list[list[str]] = []

        self.camera: Camera | None = None
        """The camera to capture images from; does not capture if set to ``None``."""

        self.ongoing_compressions: list[str] = []
        """List of video files that are currently being compressed."""

        rosys.on_repeat(self._capture, 0.01)
        self.frame_info_builder: Callable[[RosysImage], str | None] = lambda image: None

    async def _capture(self) -> None:
        if self.camera is None:
            return
        if rosys.time() - self.last_capture_time < 1 / self.capture_rate:
            return
        images = self.camera.get_recent_images(timespan=2)
        if self.avoid_broken_images:
            images = [i for i in images if not i.is_broken and i.time < rosys.time() - 0.1]
        if images:
            self.last_capture_time = rosys.time()
            await self.save(images[-1])

    async def save(self, image: RosysImage, overlay: str | None = None) -> None:
        """Captures an image to be used in video."""
        await rosys.run.cpu_bound(_save_image,
                                  image,
                                  STORAGE_PATH,
                                  (self.width, self.height),
                                  self._notifications.pop(0) if self._notifications else [],
                                  self.frame_info_builder(image),
                                  overlay)

    def compress_video(self) -> Task:
        """Create a video from the captured images.

        Note: This method starts a background task and returns immediately.

        You can use the returned task or the ``ongoing_computations`` property to check if the video is still being compressed.
        """
        return rosys.background_tasks.create(self._run_ffmpeg(), name='timelapse ffmpeg')

    async def _run_ffmpeg(self) -> None:
        jpgs = sorted(STORAGE_PATH.glob('*.jpg'))
        if len(jpgs) < 20:
            self.log.info('very few images (%s); not creating video', len(jpgs))
            self.discard_video()
            return
        start = datetime.fromtimestamp(float(jpgs[0].stem.split('_')[0]))
        end = datetime.fromtimestamp(float(jpgs[-1].stem.split('_')[0]))
        self.log.info('creating video from %s to %s', start, end)
        duration = humanize.naturaldelta(end - start)
        await self.create_info(start.strftime(r'%d.%m.%Y %H:%M:%S'), duration, time=start.timestamp() - 1)
        id_ = start.strftime(r'%Y%m%d_%H-%M-%S_' + duration.replace(' ', '_'))
        VIDEO_PATH.mkdir(parents=True, exist_ok=True)
        target_dir = STORAGE_PATH / id_
        target_dir.mkdir(parents=True, exist_ok=True)
        target_filename = f'{id_}.mp4'
        for jpg in STORAGE_PATH.glob('*.jpg'):
            jpg.rename(target_dir / jpg.name)
        source_file = target_dir / 'source.txt'
        with source_file.open('w') as f:
            jpegs = sorted(target_dir.glob('*.jpg'))
            for i, jpg in enumerate(jpegs):
                cam = jpg.stem.split('_')[-1]
                before = jpegs[i - 1].stem.split('_')[-1] if i > 0 else None
                after = jpegs[i + 1].stem.split('_')[-1] if i < len(jpegs) - 1 else None
                if before and after and cam not in (before, after):
                    # NOTE: within sequences, we skip isolated frames from a different camera to reduce flickering
                    continue
                f.write(f"file '{jpg}'\n")
        absolute_niceness = 10 - os.nice(0)
        cmd = (
            f'nice -n {absolute_niceness} ffmpeg -hide_banner -threads 1 -f concat -safe 0 -i "{source_file}" '
            f'-s {self.width}x{self.height} -vcodec libx264 -crf 18 -preset slow -pix_fmt yuv420p -y {target_dir}/{target_filename};'
            f'mv {target_dir}/*mp4 {VIDEO_PATH};'
            f'rm -r {target_dir};'
        )
        self.log.info('video compression for %s starting:\n%s', target_filename, cmd)
        self.ongoing_compressions.append(target_filename)
        await rosys.run.sh(cmd, timeout=None, shell=True)
        self.ongoing_compressions.remove(target_filename)
        self.log.info('video compression for %s finished', target_filename)

    def discard_video(self) -> None:
        """Drop the currently recorded video data."""
        for jpg in STORAGE_PATH.glob('*.jpg'):
            jpg.unlink()

    async def create_info(self, title: str, subtitle: str, frames: int = 20, time: float | None = None) -> None:
        """Shows info on black screen with title and subtitle"""
        await rosys.run.cpu_bound(_save_info, title, subtitle, rosys.time() if time is None else time, frames)

    def notify(self, message: str, frames: int = 10) -> None:
        """Place message in the next n frames"""
        while len(self._notifications) < frames:
            self._notifications.append([])
        for i in range(frames):
            self._notifications[i].append(message)


def _save_image(image: RosysImage,
                path: Path,
                size: tuple[int, int],
                notifications: list[str],
                frame_info: str | None = None,
                overlay: str | None = None) -> None:
    assert image.data is not None
    img = Image.open(io.BytesIO(image.data))
    draw = ImageDraw.Draw(img)
    x = y = 20
    frame_info = ', ' + frame_info if frame_info else ''
    _write(f'{datetime.fromtimestamp(image.time):%Y-%m-%d %H:%M:%S}' + frame_info, draw, x, y)
    for message in notifications:
        y += 30
        _write(message, draw, x, y)
    if overlay:
        style = 'position:absolute;top:0;left:0;pointer-events:none'
        viewbox = f'0 0 {image.size.width} {image.size.height}'
        svg_image = svg2png(bytestring=f'<svg style="{style}" viewBox="{viewbox}">{overlay}</svg>')
        overlay_img = Image.open(io.BytesIO(svg_image))
        img.paste(overlay_img, (0, 0), overlay_img)
    img.resize(size).save(path / f'{image.time:.3f}_{image.camera_id.replace(":", "-").upper()}.jpg', 'JPEG')


def _write(text: str, draw: ImageDraw.ImageDraw, x: int, y: int) -> None:
    # shadow
    draw.text((x - 1, y - 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x + 1, y - 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x - 1, y + 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    draw.text((x + 1, y + 1), text, font=IMAGE_FONT, fill=(0, 0, 0))
    # actual text
    draw.text((x, y), text, font=IMAGE_FONT, fill=(255, 255, 255))


def _save_info(title: str, subtitle: str, time: float, frames: int) -> None:
    for i in range(frames):
        img = Image.new('RGB', (1600, 1200), 'black')
        draw = ImageDraw.Draw(img)
        font = BIG_COVER_FONT if len(subtitle) < 25 and len(title) < 25 else SMALL_COVER_FONT

        title_box = draw.textbbox((0, 0), title, font=font)
        title_x = (img.width - title_box[2]) / 2
        title_y = (img.height / 2) - 200 - (title_box[3] / 2)
        draw.text((title_x, title_y), title, font=font, fill='white')

        subtitle_box = draw.textbbox((0, 0), subtitle, font=font)
        subtitle_x = (img.width - subtitle_box[2]) / 2
        subtitle_y = (img.height / 2) + 100 - (subtitle_box[3] / 2)
        draw.text((subtitle_x, subtitle_y), subtitle, font=font, fill='white')

        img.save(STORAGE_PATH / f'{time:.3f}{i:03}.jpg')
