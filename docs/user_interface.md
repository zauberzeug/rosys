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

## 3D Scene

### Robot and Shape

It is often desired to visualize all the robot's information about the world.
To do so you can create a 3d scene with [NiceGUI](https://nicegui.io).
RoSys provides a `robot_object` to render and update the robot:

```python hl_lines="10-12"
{!src/robot_shape.py [ln:5-16] !}
```

### Click Handler

You can also pass a click handler to the 3d scene.
Here is a full example example for driving to a point on the ground by starting the built-in automation called `drive_to`:

```python hl_lines="11 18"
{!src/scene_on_click.py!}
```

![Click Handler](scene_on_click.webp){: style="width:60%"}

## Automation Controls

Required for [safety](safety.md) and good usability: automation processes only begin after the user an actively requests it.
Also the user should always be able to pause/resume and stop an ongoing automation.
While you could write your own UI, RoSys already provides a ready-made set of elements with `rosys.ui.automation_controls()`.
Building on the [click handler](user_interface.md#click-handler) example above we can add these easily:

```python hl_lines="3"
{!src/scene_on_click_with_automation_controls.py [ln:21-23]!}
```

The `ui.row()` context arranged the control buttons in a row.
