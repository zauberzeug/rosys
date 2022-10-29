# RoSys - The Robot System

RoSys provides an easy-to-use robot system.
Its purpose is similar to [ROS](https://www.ros.org/).
But RoSys is fully based on modern web technologies and focusses on mobile robotics.

See full documentation at [rosys.io](https://rosys.io/).

Currently RoSys is mostly tested and developed on the [Zauberzeug Robot Brain](https://www.zauberzeug.com/robot-brain.html) which uses [Lizard](https://lizard.dev/) for communication with motors, sensors and other peripherals.
But the software architecture of RoSys also allows you to write your own modules if you prefer another industrial PC or setup.

## Principles

### All Python

Business logic is written in pure Python.
Computation-heavy tasks are wrapped in processes, accessed through websockets or called via C++ bindings.
Like you would do in any other Python program.

### Modularity

You can structure your code as you please.
RoSys provides it's magic without assuming a specific file structure, config files or enforced naming.

### Async

Thanks to [asyncio](https://docs.python.org/3/library/asyncio.html) you can write your business logic without locks and mutexes.
The execution is [parallel but not concurrent, which makes it really fast](https://realpython.com/python-concurrency/).
In real case scenarios this is much faster than ROS with it's multiprocessing architecture which requires a lot of inter-process communication.

### Web UI

Most machines need some kind of human interaction.
We made sure your robot can be operated fully off the grid with any web browser by incorporating [NiceGUI](https://nicegui.io/).
It is also possible to proxy the user interface through a gateway for remote operation.

### Simulation

Robot hardware is often slower than your own computer.
To rapidly test out new behavior and algorithms, RoSys provides a simulation mode.
Here, all hardware is mocked and can even be manipulated to test wheel blockages and similar.

### Testing

You can use [pytest](https://docs.pytest.org/) to write high-level integration tests.
It is based on the above-described simulation mode and accelerates the robot's time for super fast execution.

## Architecture and Features

### Modules

RoSys modules basically are Python modules which encapsulate certain functionality.
They can hold their own state, register lifecycle hooks, run methods repeatedly and subscribe to or raise [events](#events).
Most modules depend on other modules.

### Lifecycle Hooks And Loops

Modules can register functions for being called `on_startup` or `on_shutdown` as well as repeatedly with a given interval.

### Events

Modules can provide events to allow coupling otherwise separated modules of the system.
For example on module might read sensor data and raise an event `NEW_SENSOR_DATA`, without knowing of any consumers.
Another module can register on `NEW_SENSOR_DATA` and act accordingly when being called.

### Automations

RoSys provides an `Automator` module for running "automations".
Automations are coroutines that can not only be started and stopped, but also paused and resumed, e.g. using `AutomationControls`.

### Persistence

Modules can register backup and restore methods to read and write their state to disk.

### RoSys Time

If you want to delay the execution, you should invoke `await rosys.sleep(seconds: float)`.
This causes to wait until the _RoSys time_ has elapsed the desired amount of time.
In pytests the RoSys time is simulated and can advance much faster if no CPU-intensive operation is performed.

### Threading And Multiprocessing

Not every piece of code is already using asyncio.
The actor class provides convenience functions for IO and CPU bound work.

IO Bound:
If you need to read from an external device or use a non-async HTTP library like [requests](https://requests.readthedocs.io/),
you should wrap the code in a function and await it with `await rosys.run.io_bound(...)`.

CPU Bound:
If you need to do some heavy computation and want to spawn another process,
you should wrap the code in a function and await it with `await rosys.run.cpu_bound(...)`.

### Safety

Python is fast enough for most high level logic, but has no realtime guarantees.
Safety-relevant behavior should therefore be written in [Lizard](https://lizard.dev/) and executed on a suitable microprocessor.
The microprocessor governs the hardware of the robot and must be able to perform safety actions like triggering emergency hold etc.
We suggest you use an industrial PC with an integrated controller like the [Zauberzeug Robot Brain](https://www.zauberzeug.com/robot-brain.html).
It provides a Linux system with AI acceleration to run RoSys, two integrated [ESP32](https://www.espressif.com/en/products/socs/esp32) to run Lizard and six I/O sockets with up to 24 GPIOs for digital I/Os, CAN, RS485, SPI, I2C, ... with a software controllable ENABLE switch.

### User Interface

RoSys plays very well with [NiceGUI](https://nicegui.io/) and provides additional robot-related UI elements.
NiceGUI is a high-level web UI framework on top of [JustPy](https://justpy.io/).
This means you can write all UI code in Python.
The state is automatically reflected in the browser through WebSockets.
RoSys can also be used with other user interfaces or interaction models if required, for example a completely app-based control through Bluetooth Low Energy with Flutter.

### Notifications

Modules can notify the user through `rosys.notify('message to the user')`.
When using NiceGUI, the notifications will show as snackbar messages.
The history of notifications is stored in the list `rosys.notifications`.
