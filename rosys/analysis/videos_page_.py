from collections import defaultdict
from datetime import datetime
from pathlib import Path

import watchfiles
from nicegui import background_tasks, run, ui

VIDEO_FILES = Path('~/.rosys/timelapse/videos').expanduser()


class VideosPage:

    def __init__(self) -> None:

        @ui.page('/videos', title='Videos')
        async def page():
            @ui.refreshable
            async def videos(selected_date: str):
                all_videos = await get_all_videos()
                with ui.card().tight().props('flat bordered'):
                    with ui.card_section().classes('w-full bg-primary text-white'):
                        ui.label(_format_date(selected_date)).classes('text-xl')
                    with ui.card_section().classes('w-full'):
                        if selected_date not in all_videos:
                            ui.label('No videos for this date')
                        else:
                            with ui.list().classes('items-center'):
                                for mp4 in all_videos[selected_date]:
                                    video_date, video_duration = format_video_name(mp4)
                                    with ui.item().classes('p-0'):
                                        with ui.item_section().props('avatar'):
                                            ui.button(icon='download', on_click=lambda mp4=mp4: ui.download(mp4)) \
                                                .props('flat').tooltip('download')
                                        with ui.item_section().classes('min-w-24'):
                                            ui.item_label(video_date)
                                            ui.item_label(video_duration).props('caption')
                                        with ui.item_section().classes('pl-1'):
                                            with ui.button(icon='more_vert').props('flat fab-mini'):
                                                with ui.menu():
                                                    with ui.menu_item(on_click=lambda mp4=mp4: mp4.unlink()):
                                                        with ui.item_section().props('side'):
                                                            ui.icon('delete', color='negative')
                                                        ui.item_section('delete')

            async def watch_videos():
                async for _ in watchfiles.awatch(VIDEO_FILES):
                    videos.refresh()

            ui.label('Timelapse Videos').classes('text-2xl')
            with ui.row(wrap=False).classes('w-full'):
                date_picker = ui.date(value=datetime.now().date().strftime('%Y%m%d'),
                                      mask='YYYYMMDD',
                                      on_change=lambda e: videos.refresh(e.value))
                await videos(date_picker.value)

            background_tasks.create(watch_videos(), name='watch videos files')


def _format_date(date: str) -> str:
    return datetime.strptime(date, '%Y%m%d').strftime('%d.%m.%Y')


def format_video_name(video: Path) -> tuple[str, str]:
    parts = video.stem.split('_')
    time = parts[1].replace('-', ':')
    return time, f'{parts[2]} {parts[3]}'


async def get_all_videos() -> dict[str, list[Path]]:
    all_videos = sorted(await run.io_bound(VIDEO_FILES.glob, '*.mp4'), reverse=True)
    video_date_dict: dict[str, list[Path]] = defaultdict(list)
    for video in all_videos:
        video_date = video.stem.split('_')[0]
        video_date_dict[video_date].append(video)
    return video_date_dict
