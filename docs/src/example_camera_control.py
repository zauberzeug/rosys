#!/usr/bin/env python3
from nicegui import ui

import rosys
from rosys.vision import Camera, ConfigurableCameraMixin, RtspCamera


def add_card(camera: Camera) -> None:
    uid = camera.id
    camera_card = ui.card().tight()
    camera_cards[uid] = camera_card
    if uid not in streams:
        with camera_grid:
            with camera_card:
                streams[uid] = ui.interactive_image()
                ui.label(uid).classes('m-2')
                with ui.row():
                    ui.switch('stream').bind_value(camera, 'streaming')
                    ui.button('capture', on_click=camera.capture_image)
                if isinstance(camera, ConfigurableCameraMixin):
                    create_camera_settings_panel(camera)

    streams[uid].set_source(camera.get_latest_image_url())


def create_camera_settings_panel(camera: ConfigurableCameraMixin) -> None:
    camera_parameters = camera.get_capabilities()
    parameter_names = [parameter.name for parameter in camera_parameters]
    with ui.expansion('Einstellungen').classes('w-full').bind_enabled_from(camera, 'is_connected'):
        if isinstance(camera, RtspCamera):
            ui.label('URL').classes('text-s').bind_text_from(camera,
                                                             'url', backward=lambda x: x or 'url nicht verfügbar')
        if 'fps' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.label("FPS: ").classes('text-s')
                    ui.select(options=list(range(1, 31)), on_change=lambda event: camera.set_parameters({'fps': event.value})).classes(
                        'text-s').bind_value_from(camera, 'parameters', backward=lambda params: params['fps'])
        if 'jovision_profile' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.switch('High Quality', on_change=lambda event: camera.set_parameters(
                        {'jovision_profile': 0 if event.value else 1})).classes('text-s')
        if 'exposure' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.label("Exposure: ").classes('text-s')
                    ui.select(options=list(range(0, 255)), on_change=lambda event: camera.set_parameters({'exposure': event.value})).classes(
                        'text-s').bind_value_from(camera, 'parameters', backward=lambda params: params['exposure'])

        if 'auto_exposure' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.switch('Auto Exposure', on_change=lambda event: camera.set_parameters(
                        {'auto_exposure': event.value})).bind_value_from(camera, 'parameters', backward=lambda params: params['auto_exposure']).classes('text-s')
        if 'color' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.label("Color: ").classes('text-s')
                    with ui.button(icon='colorize'):
                        ui.color_picker(on_pick=lambda event: camera.set_parameters(
                            {'color': event.color})).classes('text-s')


def update_camera_cards():

    for provider in [rtsp_camera_provider, usb_camera_provider, simulated_camera_provider]:
        for camera in provider.cameras.values():
            add_card(camera)


streams: dict[str, ui.interactive_image] = {}
camera_cards: dict[str, ui.card] = {}
camera_grid = ui.row()

rtsp_camera_provider = rosys.vision.RtspCameraProvider()
usb_camera_provider = rosys.vision.UsbCameraProvider()
simulated_camera_provider = rosys.vision.SimulatedCameraProvider()

ui.timer(.01, update_camera_cards)

simulated_camera_provider.add_cameras(1)

ui.run(title='RoSys', port=8080)
