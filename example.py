

import rosys

from rosys.hardware import RobotBrain, ESPPins
from rosys.hardware.communication import SerialCommunication
from nicegui import ui


# async def startup() -> None:


# rosys.on_startup(startup)
esp_pins: ESPPins


async def state_switcher() -> None:
    esp_pins.gpio_states[1].level = not esp_pins.gpio_states[1].level
    esp_pins.gpio_states[1].is_pullup = not esp_pins.gpio_states[1].is_pullup


serial_communication = SerialCommunication(device_path='/dev/tty.usbserial-0001')
robot_brain = RobotBrain(serial_communication)
robot_brain.developer_ui()
esp_pins = ESPPins(name='esp', robot_brain=robot_brain)
esp_pins.developer_ui()


rosys.on_repeat(state_switcher, 2.0)
ui.run()
