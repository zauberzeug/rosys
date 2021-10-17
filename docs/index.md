# RoSys

RoSys is an easy-to-use robot system based on modern web technologies.
The purpose is very simlar to [ROS](https://www.ros.org/).
RoSys is completely built on existing web technologies and focus on mobile robotics.

## Features

**All Python**
: Business logic is wired in Python while computation-heavy tasks are encapsulated through websockets or bindings.

**Shared State**
: All code can access and manipulate a shared, typesafe global state -- this does not mean it should.
Good software design is still neccessary.
But it is much easier to do if you do not have to perform serialization all the time.

**No Threading**
: Thanks to [asyncio](https://docs.python.org/3/library/asyncio.html) you can write the business logic without locks and mutex mechanisms.
The running system feels like everything is happening in parallel. But each code block is executed one after another through an event queue and yields execution as soon as it waits for I/O or heavy computation.
Which is still executed in threads to not block the rest of the business logic.

**Web UI**
: Most machines need some kind of human interaction.
We made sure your robot can be operated fully off the grid with any web browser by incororporating [NiceGUI](https://nicegui.io/).
It's also possible to proxy the user interface through a gateway for remote operation.

**Simulation**
: Robot hardware is often slower than your own computer.
Therefore RoSys supports a simulation mode for rapid development.
To get maximum performance the current implementation does not run a full physics engine.

**Testing**
: You can use pytest to write high-level integration tests.
It is based on the above-described simulation mode and accelerates the robot's time for super fast execution.

!!! note

    Currently RoSys is mostly tested and developed on the [Zauberzeug Robot Brain](https://www.zauberzeug.com/product-robot-brain.html) which uses [Lizard](https://lizard.dev/) for communication with motors, sensors and other peripherals.
    But the software architecture of RoSys also allows you to write your own actors if you prefer another industrial PC or setup.
