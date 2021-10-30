# Getting Started

First [install RoSys](installation.md) with pip or Docker.
Then create a directory to host your code and put it under version control.
Name your entry file `main.py` and add the following content:

```Python
{!../main.py!}
```

If you launch the program, your browser will open the url <http://0.0.0.0:8080/> and present a 3d view:

![Screenshot](getting_started_01.png){: style="width:60%"}

## Explenations

### Imports

The [User Interface](user_interface.md) is build with [NiceGUI](https://nicegui.io).
The imports must be stated seperately to make it possible to run RoSys without it.

### Wireing

As you can read up in the ["Basic Concepts" section](basic_concepts.md) RoSys provides a runtime to manage the actors which operate on the world. The command `rosys.ui.configure(ui, runtime)` connects the user interface with the runtime.

### Keyboard Control

By calling `rosys.ui.keyboard_control()` you create a keyboard event listener which will steer the robot.
There are also other possibilities of steering the robot like a [Joystick](user_interface.md#joystick).

### 3D Scene

As it is common for groups of UI elements in NiceGUI, a 3d scene is created by using context through Python's `with`-Statement.
Every command "inside" is applied the created scene. Here a 3d reperesentation of the robot is created (`rosys.ui.robot_object()`). Then the `ui.timer` ensures it's position is updated every 50 ms.
An `ui.label` is used afterwards to explain the keyboard interaction. Note: the lable is on the same intendation level as the `ui.scene` object, not within. See [NiceGUI](https://nicegui.io) for an complete API reference.

### Start

NiceGUI provids an `ui.run` command which launches the webserver and presents the interface as configured above.
If you modify the code and automatic reload is triggered. Very convinient.
