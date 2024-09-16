from nicegui import ui

from rosys.hardware import ESPPins, RobotBrain
from rosys.hardware.communication import SerialCommunication

esp_pins: ESPPins

serial_communication = SerialCommunication(device_path='/dev/tty.usbserial-0001')
robot_brain = RobotBrain(serial_communication)
robot_brain.developer_ui()

with ui.row():
    with ui.card().style('min-width: 200px; background-color: #3E63A6; color: white;'):
        esp_pins = ESPPins(name='core', robot_brain=robot_brain)
        esp_pins.developer_ui()

    # with ui.card().style('min-width: 200px; background-color: #3E63A6; color: white;'):
    esp_pins = ESPPins(name='p0', robot_brain=robot_brain)
    esp_pins.developer_ui()

ui.run()
