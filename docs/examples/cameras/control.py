#!/usr/bin/env python3
import logging

from nicegui import ui

import rosys.vision


def resolution_to_str(resolution: tuple[int, int] | None) -> str | None:
    return f'{resolution[0]}x{resolution[1]}' if resolution else None


def str_to_resolution(text: str) -> tuple[int, int]:
    width, height = text.split('x')
    return int(width), int(height)


def create_camera_settings_panel(camera: rosys.vision.ConfigurableCamera) -> None:
    camera_parameters = camera.get_capabilities()
    parameter_names = [parameter.name for parameter in camera_parameters]
    with ui.expansion('Settings').classes('w-full') \
            .bind_enabled_from(camera, 'is_active'):
        if isinstance(camera, rosys.vision.RtspCamera):
            ui.label('URL') \
                .bind_text_from(camera, 'url',
                                backward=lambda x: x or 'URL not available')
        if 'fps' in parameter_names:
            with ui.card(), ui.row():
                ui.label('FPS:')
                ui.select(options=list(range(1, 31)),
                          on_change=lambda e: camera.set_parameters({'fps': e.value})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: params['fps'])
        if 'resolution' in parameter_names:
            resolutions = [(1920, 1080), (1280, 720), (800, 600), (640, 480), (480, 360)]
            with ui.card(), ui.row():
                ui.label('Resolution:')
                ui.select(options=[resolution_to_str(r) for r in resolutions],
                          on_change=lambda e: camera.set_parameters({'resolution': str_to_resolution(e.value)})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: resolution_to_str(params.get('resolution')))
        if 'mirrored' in parameter_names:
            with ui.card():
                ui.switch('Mirrored',
                          on_change=lambda e: camera.set_parameters({'mirrored': e.value})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: bool(params.get('mirrored')))
        if 'substream' in parameter_names:
            with ui.card():
                ui.switch('High Quality',
                          on_change=lambda e: camera.set_parameters({'substream': 0 if e.value else 1}))
        if 'bitrate' in parameter_names:
            with ui.card(), ui.row():
                ui.label('Bitrate (kbit):')
                ui.select(options=[512, 1024, 2048, 4096, 8192],
                          on_change=lambda e: camera.set_parameters({'bitrate': e.value})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: params.get('bitrate'))
        if 'exposure' in parameter_names:
            with ui.card(), ui.row():
                ui.label('Exposure:')
                ui.select(options=list(range(0, 255)),
                          on_change=lambda e: camera.set_parameters({'exposure': e.value})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: params['exposure'])
        if 'auto_exposure' in parameter_names:
            with ui.card():
                ui.switch('Auto Exposure',
                          on_change=lambda e: camera.set_parameters({'auto_exposure': e.value})) \
                    .bind_value_from(camera, 'parameters',
                                     backward=lambda params: params['auto_exposure'])
        if 'color' in parameter_names:
            with ui.card(), ui.row():
                ui.label('Color:')
                with ui.button(icon='colorize'):
                    ui.color_picker(on_pick=lambda e: camera.set_parameters({'color': e.color}))


def root() -> None:
    streams: dict[str, ui.interactive_image] = {}
    camera_grid = ui.row()

    def add_card(camera: rosys.vision.Camera) -> None:
        uid = camera.id
        if uid not in streams:
            print(f'adding card for {uid}')
            with camera_grid, ui.card().tight():
                streams[uid] = ui.interactive_image()
                ui.label(uid).classes('m-2')
                with ui.row():
                    ui.button('connect', on_click=camera.connect) \
                        .bind_enabled_from(camera, 'is_active', backward=lambda active: not active)
                    ui.button('disconnect', on_click=camera.disconnect) \
                        .bind_enabled_from(camera, 'is_active')
                if isinstance(camera, rosys.vision.ConfigurableCamera):
                    create_camera_settings_panel(camera)

        streams[uid].set_source(camera.get_latest_image_url())

    def update_camera_cards() -> None:
        for provider in providers:
            for camera in provider.cameras.values():
                add_card(camera)

    ui.timer(0.1, update_camera_cards)


logging.basicConfig(level=logging.INFO)

simulated_camera_provider = rosys.vision.SimulatedCameraProvider()
simulated_camera_provider.add_cameras(1)
providers: list[rosys.vision.CameraProvider] = [
    rosys.vision.RtspCameraProvider(),
    rosys.vision.MjpegCameraProvider(),
    rosys.vision.UsbCameraProvider(),
    simulated_camera_provider,
]

# without a root function, NiceGUI re-executes this script per page and creates the providers again
ui.run(root, title='RoSys', port=8080)
