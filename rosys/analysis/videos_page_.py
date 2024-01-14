from pathlib import Path

import watchfiles
from nicegui import background_tasks, run, ui

VIDEO_FILES = Path('~/.rosys/timelapse/videos').expanduser()


class VideosPage:

    def __init__(self) -> None:
        @ui.page('/videos', title='Videos')
        async def page():
            @ui.refreshable
            async def videos():
                for mp4 in sorted(await run.io_bound(VIDEO_FILES.glob, '*.mp4'), reverse=True):
                    with ui.row():
                        with ui.card().tight():
                            video = ui.video(mp4).classes('w-[800px]')
                            src = video._props['src']  # pylint: disable=protected-access
                        with ui.column():
                            ui.button(on_click=lambda mp4=mp4: mp4.unlink()).props('icon=delete flat')
                            ui.button(on_click=lambda src=src: ui.download(src)).props('icon=download flat')

            async def watch_videos():
                async for _ in watchfiles.awatch(VIDEO_FILES):
                    videos.refresh()

            await videos()
            background_tasks.create(watch_videos(), name='watch videos files')
