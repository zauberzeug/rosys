# Steering

The following example simulates a robot that can be steered using keyboard controls or a joystick.

```python hl_lines="12-13"
{!src/example_steering.py !}
```

![Joystick](steering.png){: style="width:50%"}

Keyboard Control
: By adding a `KeyboardControl` to the user interface you enable steering the robot with the keyboard.
Press the arrow keys while holding the SHIFT key to steer the robot.
You can also modify the speed of the robot by pressing the a number key.
Use the optional parameter `default_speed` to change the initial value.

Joystick
: When operating from a mobile phone, you can use a `Joystick` to create a UI element with touch control.
You can drive the robot by dragging the mouse inside the top left square.
