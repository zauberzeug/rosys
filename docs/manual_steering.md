# Manual Steering

## Keyboard Control

By calling `rosys.ui.keyboard_control()` you enable steering the robot with the keyboard (see [Getting Started](getting_started.md) for a full example).
Press the arrow keys to steer the robot while holding the SHIFT key down.
You can also modify the speed of the robot by pressing the a number key.
Use the optional parameter `default_speed` to change the inital value.

## Joystick

When operating from a mobile phone, you can use `rosys.ui.joystick()` to create a UI element with touch control.
You can drive the robot by dragging the mouse inside the top left square:

![Joystick](joystick.png){: style="width:50%"}
