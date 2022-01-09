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

Afterwards you can use the UI elements provided by RoSys like [automation controls](../features/automation_controls.md), [3d scene](../features/3d_scene.md), [cameras](../features/cameras.md) or [joystick](../features/manual_steering.md#joystick).
