#!/usr/bin/env python3
"""Multiple GMSL2/FPD-Link cameras on an NVIDIA Jetson, shown through `GmslCameraProvider`.

GMSL cameras are registered explicitly by their Argus ``sensor_id`` (the GMSL port, which
maps 1:1 to ``/dev/video<sensor_id>``); there is no auto-discovery (see `GmslCameraProvider`).

Note for Jetson: do NOT also run `UsbCameraProvider` here -- on a Jetson the GMSL sensors
appear as ``/dev/video*`` nodes, so the USB provider would grab them (and a V4L2 grab can
block Argus). Likewise `RtspCameraProvider`/`MjpegCameraProvider` arp-scan the network.
"""
from nicegui import ui

import rosys

SENSOR_IDS = [0, 1]  # the Argus sensor-ids (= GMSL ports = /dev/videoN) with a camera connected

provider = rosys.vision.GmslCameraProvider()
for sensor_id in SENSOR_IDS:
    provider.add_camera(rosys.vision.GmslCamera(id=f'gmsl-{sensor_id}', sensor_id=sensor_id))

streams: dict[str, ui.interactive_image] = {}
grid = ui.row()


def update() -> None:
    for camera in provider.cameras.values():
        if camera.id not in streams:
            with grid, ui.card().tight():
                streams[camera.id] = ui.interactive_image()
                ui.label(camera.id).classes('m-2')
                ui.switch('Auto exposure', value=True,
                          on_change=lambda e, c=camera: c.set_parameters({'auto_exposure': e.value}))
                ui.button('Long exposure', on_click=lambda _, c=camera:
                          c.set_parameters({'auto_exposure': False, 'exposure': 0.25, 'fps': 4}))
        streams[camera.id].set_source(camera.get_latest_image_url())


ui.timer(0.1, update)

ui.run(title='RoSys GMSL cameras')
