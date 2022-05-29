#!/usr/bin/env python3
import os
from glob import glob
from pathlib import Path

from nicegui import ui
from starlette import responses


class VideosPage:

    def __init__(self) -> None:
        with ui.page('/videos'):
            for mp4 in glob(os.path.expanduser('~/.rosys/timelapse/*mp4')):
                name = Path(mp4).stem
                with ui.page(f'/videos/{name}', name) as page:
                    ui.html(self.create_video_tag(name))
                ui.link(name, page)

    @ui.get('/videos/{name}/mp4')
    def produce_plain_response(name: str):
        return responses.FileResponse(os.path.expanduser(f'~/.rosys/timelapse/{name}.mp4'))

    def create_video_tag(name: str) -> str:
        return f'''
<video style="
  position: absolute;
  left: 50%;
  top: 50%;
  max-height: 100%;
  max-width: 100%;
  transform: translate(-50%, -50%);
" autoplay muted loop controls>
  <source src="/videos/{name}/mp4" type="video/mp4">
Your browser does not support the video tag.
</video>
'''
