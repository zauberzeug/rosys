from pathlib import Path

from nicegui import app, ui
from starlette import responses

PATH = Path('~/.rosys/timelapse/videos').expanduser()


class VideosPage:

    def __init__(self) -> None:
        @ui.page('/videos', title='Videos')
        def page():
            def update_list() -> None:
                video_list.clear()
                with video_list:
                    for mp4 in sorted(PATH.glob('*.mp4'), reverse=True):
                        ui.link(mp4.stem.replace('_', ' ').replace('-', ':'), f'/timelapse/{mp4.name}')
            video_list = ui.column()
            ui.timer(5, update_list)

        @app.get('/timelapse/{filename}')
        def get_timelapse(filename: str) -> responses.FileResponse:
            return responses.FileResponse(PATH / filename)
