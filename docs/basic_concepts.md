# Basic Concepts

## World

All state is stored in a data class called `World`.
It not only contains the robots position, time, obstacles, etc but also images and other sensor data.
This makes it easy to persist and restore the full state and simplifies the modularization of different Actors.

## Actors

Actors encapsulate behaviour.
Each should perform a specific task which they perform by reading and manipulating the world.
For example: communication with a sensor, AI detection, monitoring battery level or similar.
Actors can specify a frequency in which they want get called.
Alternativly, Actors can be chained through `follow_ups`.
These are helpful for example if one Actor fetches an image and another one should process it as soon as it's available.

## Automations

RoSys provides an Actor called `runtime.automator` which receives instruction sequences.
These Automations normally contains machine commands followed by conditions which can be awaited.
For example there are alreay build-in automations like steering the robot along a spline.
