# Basic Concepts

## World

All state is stored in a data class called `World`.
It not only contains the robots position, time, obstacles, etc but also images and other sensor data.
This makes it easy to persist and restore the full state as needed and simplifies the modularization of different Actors.

## Actors

Actors encapsulate behaviour.
They should perform a specific goal which they perform by reading and manipulating the world.
For example communication with a sensor, decision making after AI detections or similar.
Actors can specify a frequency in which they want get called.
Alternativly Actors can be chained through `follow_ups`.
This is helpful for example if you want to process an image in one Actor after an other has downloaded it successfully.

## Automations

RoSys provides an Actor `runtime.automator` to write instruction-sequences.
These Automations normally contains machine commands followed by conditions which can be awaited.
For example there are alreay build in automations which steer the robot along a spline or find it's way around obstacles.

## User Interface

RoSys plays very well with [NiceGUI](https://nicegui.io/) and provides aditional robot-related components through the `rosys.ui` package.
NiceGUI is a high-level web UI framework on top of [JustPy](https://justpy.io/).
This means you can write all UI code in Python.
The state is automatically reflected in the browser through websockets.
If required RoSys can also be used with other user interfaces or interaction models.
For example a completely App based contol through Bluetooth LE with Flutter.
