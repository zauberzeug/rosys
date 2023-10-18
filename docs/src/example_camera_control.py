#!/usr/bin/env python3
from nicegui import ui

import rosys
from rosys.vision import Camera, ConfigurableCameraMixin, RtspCamera

rtsp_camera_provider = rosys.vision.RtspCameraProvider()
usb_camera_provider = rosys.vision.UsbCameraProvider()
simulated_camera_provider = rosys.vision.SimulatedCameraProvider()


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
                    ui.select(options=list(range(1, 31)), on_change=lambda event: camera.update_parameters({'fps': event.value})).classes(
                        'text-s').bind_value_from(camera, backward=lambda cam: cam.get_parameters()['fps'])
        if 'jovision_profile' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.switch('High Quality', on_change=lambda event: camera.update_parameters(
                        {'jovision_profile': 0 if event.value else 1})).classes('text-s')
        if 'exposure' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.label("Exposure: ").classes('text-s')
                    ui.select(options=list(range(0, 256)), on_change=lambda event: camera.update_parameters({'exposure': event.value})).classes(
                        'text-s').bind_value_from(camera, backward=lambda cam: cam.get_parameters()['exposure'])
        if 'auto_exposure' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.switch('Auto Exposure', on_change=lambda event: camera.update_parameters(
                        {'auto_exposure': event.value})).classes('text-s')
        if 'color' in parameter_names:
            with ui.card().classes():
                with ui.row():
                    ui.label("Color: ").classes('text-s')
                    with ui.button(icon='colorize'):
                        ui.color_picker(on_pick=lambda event: camera.update_parameters(
                            {'color': event.color})).classes('text-s')


def create_ui(camera_provider):
    for uid, camera in camera_provider.cameras.items():
        assert isinstance(camera, Camera)
        if uid not in streams:
            with camera_grid:
                with ui.card().tight():
                    streams[uid] = ui.interactive_image()
                    ui.label(uid).classes('m-2')
                    with ui.row():
                        ui.switch('stream').bind_value(camera, 'streaming')
                        ui.button('capture', on_click=camera.capture_image)
                    if isinstance(camera, ConfigurableCameraMixin):
                        create_camera_settings_panel(camera)

        streams[uid].set_source(camera.get_latest_image_url())


def refresh() -> None:
    for provider in [rtsp_camera_provider, usb_camera_provider, simulated_camera_provider]:
        create_ui(provider)


streams: dict[str, ui.interactive_image] = {}
camera_grid = ui.row()
ui.timer(0.01, refresh)

simulated_camera_provider.add_cameras(1)

ui.run(title='RoSys', port=8080)
