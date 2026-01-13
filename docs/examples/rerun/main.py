#!/usr/bin/env python3
from pathlib import Path

from nicegui import ui

import rosys
from rosys.driving import Odometer
from rosys.hardware import WheelsSimulation
from rosys.vision import SimulatedCamera, SimulatedCameraProvider
from rosys.visualization import RerunLogger, RerunViewer

# Create simulated hardware
wheels = WheelsSimulation()
odometer = Odometer(wheels)
camera_provider = SimulatedCameraProvider()
camera_provider.add_camera(SimulatedCamera(id='front_cam', width=640, height=480, fps=30))

# Create RerunLogger (singleton - safe to create at module level)
rerun_logger = RerunLogger(app_id='rosys_example')


def startup():
    rerun_logger.connect_camera_provider(camera_provider)
    rerun_logger.connect_odometer(odometer)


# Build the UI


@ui.page('/')
def main_page():
    with ui.card().classes('w-full'):
        ui.label('Rerun Visualization').classes('text-xl font-bold')

        with ui.row().classes('w-full items-center'):
            ui.label('Recording:')
            recording_label = ui.label('Not recording').classes('text-gray-500')

            def toggle_recording():
                if rerun_logger.is_recording:
                    path = rerun_logger.stop_recording()
                    recording_label.text = f'Saved to {path}'
                    record_btn.text = 'Start Recording'
                else:
                    path = Path('~/recordings') / f'session_{rosys.time():.0f}.rrd'
                    rerun_logger.start_recording(path)
                    recording_label.text = f'Recording to {path}'
                    record_btn.text = 'Stop Recording'

            record_btn = ui.button('Start Recording', on_click=toggle_recording)

    with ui.card().classes('w-full h-[700px]'):
        viewer = RerunViewer(rerun_logger).classes('w-full h-full')
        viewer.set_time_window(seconds=10)


# Simulate some robot movement
async def simulate_movement():
    await wheels.drive(linear=0.1, angular=0.05)
    while True:
        await wheels.step(dt=0.02)  # Run simulation at 50Hz
        await rosys.sleep(0.02)


rosys.on_startup(simulate_movement)
rosys.on_startup(startup)

ui.run(title='RoSys Rerun Example', port=8080)
