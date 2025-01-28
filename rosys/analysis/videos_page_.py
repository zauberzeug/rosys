from collections import defaultdict
from datetime import datetime
from pathlib import Path

from nicegui import run, ui

VIDEO_FILES = Path('~/.rosys/timelapse/videos').expanduser()


class VideosPage:

    def __init__(self) -> None:

        @ui.page('/video/{name:str}')
        async def video_page(name: str) -> None:
            ui.video(VIDEO_FILES / name)

        @ui.page('/videos', title='Videos')
        async def page():
            @ui.refreshable
            def video_ui() -> None:
                with ui.card().props('flat bordered'):
                    if date.value not in all_videos:
                        ui.label('No videos for this date')
                        return
                    with ui.list().classes('items-center'):
                        for mp4 in all_videos[date.value]:
                            video_date, video_duration = format_video_name(mp4)
                            with ui.item().classes('p-0'):
                                with ui.item_section().props('avatar'):
                                    ui.button(icon='download', on_click=lambda mp4=mp4: ui.download(mp4)) \
                                        .props('flat').tooltip('download')
                                with ui.item_section().classes('min-w-24'):
                                    with ui.link(target=f'video/{mp4.name}').classes('no-underline'):
                                        ui.item_label(video_date)
                                        ui.item_label(video_duration).props('caption')
                                with ui.item_section().classes('pl-1'):
                                    with ui.button(icon='more_vert').props('flat fab-mini'):
                                        with ui.menu():
                                            def delete_video(mp4: Path = mp4) -> None:
                                                mp4.unlink()
                                                all_videos[date.value].remove(mp4)
                                                video_ui.refresh()
                                            with ui.menu_item(on_click=delete_video):
                                                with ui.item_section().props('side'):
                                                    ui.icon('delete', color='negative')
                                                ui.item_section('delete')

            ui.label('Timelapse Videos').classes('text-2xl')
            all_videos = await get_all_videos()
            today = datetime.now().strftime(r'%Y%m%d')
            with ui.row(wrap=False).classes('w-full'):
                date = ui.date(value=today, mask='YYYYMMDD', on_change=video_ui.refresh).props('flat bordered')
                date.props['options'] = [datetime.strptime(d, r'%Y%m%d').strftime(r'%Y/%m/%d') for d in all_videos]
                video_ui()


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
