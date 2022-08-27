# Hardware

You can use `SerialCommunication.is_possible()` to automatically switch between a simulation and real hardware.
The module `WheelsHardware` expects a `RobotBrain`, which controls the `SerialCommunication` with a microcontroller.
The [Zauberzeug Robot Brain](https://zauberzeug.com/robot-brain.html) is an industrial-grade controller to combine artificial intelligence with machinery.

```python
{!src/example_hardware.py !}
```

With `communication.debug_ui()` you can add some helpful UI elements for debugging the serial communication.

Furthermore, you can add a button to send the [Lizard](https://lizard.dev/) configuration to the microcontroller running the Lizard firmware.
This way it recognizes wheel commands from the `RobotBrain` module and responds accordingly.

The lizard configuration for two wheels controlled with an [ODrive](https://odriverobotics.com/) might look as follows:

```
{!src/example_hardware.txt !}
```
