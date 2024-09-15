from rosys.hardware import RobotBrain, ESPPins
from rosys.hardware.communication import SerialCommunication
from nicegui import ui

esp_pins: ESPPins

serial_communication = SerialCommunication(device_path='/dev/tty.usbserial-0001')
robot_brain = RobotBrain(serial_communication)
robot_brain.developer_ui()

with ui.row():
    esp_pins = ESPPins(name='core', robot_brain=robot_brain)
    esp_pins.developer_ui()

    esp_pins = ESPPins(name='p0', robot_brain=robot_brain)
    esp_pins.developer_ui()

ui.run()
