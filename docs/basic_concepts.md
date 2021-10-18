# Basic Concepts

## World

All state is stored in a data class called `World`.
It not only contains the robot's position, time, obstacles, etc. but also images and other sensor data.
This makes it easy to persist and restore the full state and simplifies the modularization of different actors.

## Actors

Actors encapsulate behavior.
Each actor should perform a specific task by reading and manipulating the world, for example: communication with a sensor, AI inference, monitoring battery level or similar.
Actors can specify a frequency at which they want to get called.
Alternatively, actors can be chained through `follow_ups`.
These are helpful for example if one actor fetches an image and another one should process it as soon as it is available.

## Automations

RoSys provides an actor called `runtime.automator` which receives instruction sequences.
These automations normally contain machine commands followed by conditions which can be awaited.
For example, there are already built-in automations like steering the robot along a spline.
