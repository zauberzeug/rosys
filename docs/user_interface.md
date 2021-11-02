# User Interface

RoSys plays very well with [NiceGUI](https://nicegui.io/) and provides additional robot-related components through the `rosys.ui` package.
NiceGUI is a high-level web UI framework on top of [JustPy](https://justpy.io/).
This means you can write all UI code in Python.
The state is automatically reflected in the browser through websockets.
RoSys can also be used with other user interfaces or interaction models if required, for example a completely app-based control through Bluetooth LE with Flutter.

## Configure

To make RoSys UI elements aware of the RoSys runtime you must call `rosys.ui.configure`:

```Python hl_lines="5-6"
from nicegui import ui
import rosys
import rosys.ui

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)
```

Afterwards you can use the UI elements provided by RoSys (see below).

## Keyboard Control

By calling `rosys.ui.keyboard_control()` you enable steering the robot with the keyboard (see [Getting Started](getting_started.md) for a full example).
Press the arrow keys to steer the robot while holding the SHIFT key down.
You can also modify the speed of the robot by pressing the a number key.
Use the optional parameter `default_speed` to change the inital value.

## Joystick

When operating from a mobile phone, you can use `rosys.ui.joystick()` to create a UI element with touch control.
You can drive the robot by dragging the mouse inside the top left square:

![Joystick](joystick.png){: style="width:50%"}

## 3D Scene

### Robot and Shape

It is often desired to visualize all the robot's information about the world.
To do so you can create a 3d scene with [NiceGUI](https://nicegui.io).
RoSys provides a `robot_object` to render and update the robot:

```python hl_lines="10-12"
{!robot_shape.py [ln:5-16] !}
```

### Click Handler

You can also pass a click handler to the 3d scene.
Here is a full example example for driving to a point on the ground by starting the built-in automation called `drive_to`:

```python hl_lines="11 18"
{!scene_on_click.py!}
```

![Click Handler](scene_on_click.webp){: style="width:60%"}
