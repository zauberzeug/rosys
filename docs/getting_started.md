# Getting Started

First [install RoSys](installation.md) with pip or Docker.
Then create a directory to host your code and put it under version control.
Name your entry file `main.py` and add the following content:

```Python
{!../main.py!}
```

If you launch the program, your browser will open the url <http://0.0.0.0:8080/> and present a 3d view:

![Screenshot](getting_started_01.png){: style="width:60%"}

## Explanations

### Imports

The [User Interface](architecture/user_interface.md) is built with [NiceGUI](https://nicegui.io).
The imports must be stated separately to make it possible to run RoSys without it.

### Setup

As you can read up in the ["Architecture" chapter](architecture/architecture_overview.md) RoSys provides a runtime to manage the actors which operate on the world.
The command `rosys.ui.configure(ui, runtime)` connects the user interface with the runtime.

### Keyboard Control

By calling `rosys.ui.keyboard_control()` you create a keyboard event listener which will steer the robot.
There are also other possibilities of steering the robot like a [Joystick](architecture/user_interface.md#joystick) or [clicking in the 3d scene](architecture/user_interface.md#click-handler).

### 3D Scene

As it is common for groups of UI elements in NiceGUI, a 3d scene is created by using context through Python's `with` statement.
Every command "inside" is applied to the created scene.
Here a 3d representation of the robot is created (`rosys.ui.robot_object()`).
Then the `ui.timer` ensures its position is updated every 50 ms.
A `ui.label` is used afterwards to explain the keyboard interaction.
Note: The label is on the same intendation level as the `ui.scene` object, not within.
See [NiceGUI](https://nicegui.io) for a complete API reference.

### Start

NiceGUI provides a `ui.run` command which launches the webserver and presents the interface as configured above.
If you modify the code, an automatic reload is triggered.
Very convenient but can be deactivated by passing `reload=False`.
