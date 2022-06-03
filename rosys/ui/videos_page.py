import os
from glob import glob
from pathlib import Path

from nicegui import ui
from starlette import responses


class VideosPage:

    def __init__(self) -> None:
        with ui.page('/videos'):
            self.list = ui.column()
            ui.timer(5, self.update_list)

    def update_list(self):
        self.list.clear()
        with self.list:
            for mp4 in sorted(glob(os.path.expanduser('~/.rosys/timelapse/videos/*mp4')), reverse=True):
                name = Path(mp4).stem
                with ui.page(f'/videos/{name}', name) as page:
                    ui.html(self.create_video_tag(name))
                ui.link(name.replace('_', ' ').replace('-', ':'), page)

    @ui.get('/timelapse/{name}')
    def produce_plain_response(name: str):
        return responses.FileResponse(os.path.expanduser(f'~/.rosys/timelapse/videos/{name}.mp4'))

    def create_video_tag(self, name: str) -> str:
        return f'''
<video style="
  position: absolute;
  left: 50%;
  top: 50%;
  max-height: 100%;
  max-width: 100%;
  transform: translate(-50%, -50%);
" autoplay muted loop controls>
  <source src="timelapse/{name}" type="video/mp4">
Your browser does not support the video tag.
</video>
'''
