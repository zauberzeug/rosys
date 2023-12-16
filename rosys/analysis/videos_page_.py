from pathlib import Path

from nicegui import background_tasks, ui
from watchfiles import awatch

VIDEO_FILES = Path('~/.rosys/timelapse/videos').expanduser()


class VideosPage:

    def __init__(self) -> None:
        @ui.page('/videos', title='Videos')
        def page():

            @ui.refreshable
            def videos():
                for mp4 in sorted(VIDEO_FILES.glob('*.mp4'), reverse=True):
                    with ui.row():
                        with ui.card().tight():
                            ui.video(mp4).classes('w-[800px]')
                        with ui.column():
                            ui.button(on_click=lambda mp4=mp4: mp4.unlink()).props('icon=delete flat')
                            ui.button(on_click=lambda mp4=mp4: ui.download(mp4)).props('icon=download flat')

            async def watch_videos():
                async for _ in awatch(VIDEO_FILES):
                    videos.refresh()

            videos()
            background_tasks.create(watch_videos(), name='watch videos files')
