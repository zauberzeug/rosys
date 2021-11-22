# Basic Concepts

## World

All state is stored in a data class called [World](world.md).
It not only contains the robot's position, time, obstacles, etc. but also images and other sensor data.
This makes it easy to persist and restore the full state and simplifies the modularization of different actors.

## Actors

[Actors](actors.md) encapsulate behavior.
Each actor performs a specific task by reading and manipulating the world and emitting events, for example:
communication with a sensor, AI inference, monitoring battery level or similar.
Actors can specify a frequency at which they want to get called.

!!! note

    The "robot" is not an actor but rather represented by an object in the world.
    Actors like the odometer or steerer use and transform these data to update this representation regulary.

## Events

[Events](events.md) allow actors to signal a notable new state (most of the time written to the world).
Other actors or ui components can register for these events to act upon them.
This is helpful for example if one actor fetches an image and another one should process it as soon as it is available.

## Runtime

The runtime manages the world and all actors.
It also provides safety functionality like starting/stopping automations (see below), logging and error handling.

## Automations

RoSys provides an actor which is accessed through `runtime.automator`.
The automator can receive instruction sequences which we call "automations".
Normally they contain machine commands followed by conditions which can be awaited before proceeding with the next commands.
For example, there are already built-in automations for steering the robot along a spline.
