#!/usr/bin/env python3
import os
from typing import Any

from nicegui import ui

import rosys


class MySteerer(rosys.persistence.Persistable, rosys.driving.Steerer):

    def backup_to_dict(self) -> dict[str, Any]:
        print('Backup')
        return {'speed_scaling': self.speed_scaling}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        print('Restore')
        self.speed_scaling = data['speed_scaling']


shape = rosys.geometry.Prism.default_robot_shape()
wheels = rosys.hardware.WheelsSimulation()
steerer = MySteerer(wheels).persistent()
odometer = rosys.driving.Odometer(wheels)
robot = rosys.hardware.RobotSimulation([wheels])


@ui.page('/')
def page():
    rosys.driving.keyboard_control(steerer)
    with ui.row():
        with ui.scene(width=600, height=400):
            rosys.driving.robot_object(shape, odometer)
        with ui.column(align_items='stretch'):
            rosys.driving.joystick(steerer, size=50, color='blue')
            with ui.row():
                ui.label().bind_text_from(wheels, 'linear_velocity', lambda v: f'{v:.2f} m/s')
                ui.label().bind_text_from(wheels, 'angular_velocity', lambda v: f'{v:.2f} rad/s')
            ui.slider(min=0, max=5).props('label-always') \
                .bind_value(steerer, 'speed_scaling') \
                .on_value_change(steerer.request_backup)
            ui.button('Restart RoSys', on_click=lambda: os.utime('main.py')).props('flat')
            rosys.persistence.export_button().props('flat icon="download"')
            rosys.persistence.import_button().props('flat icon="upload_file"')


ui.run(title='RoSys - Persistence')
