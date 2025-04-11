# Hardware

The other examples use simulated hardware for simplicity and easy execution on any development system.
To be able to control real hardware we recommend to derive a `Simulation` and `Hardware` version from a shared interface.
Depending on your environment you can then instantiate the correct implementation without bothering with it in the rest of your code.

## Custom Implementation

For a differential-steering controlled robot, RoSys offers
a [`Wheels`](../../reference/rosys/hardware.md#rosys.hardware.Wheels) base class plus
a [`WheelsSimulation`](../../reference/rosys/hardware.md#rosys.hardware.WheelsSimulation).
The following example illustrates how to implement a `CustomWheelsHardware` module that derives from `Wheels`, reads the currrent velocity regularly and can be steered with linear and angular velocity.

```python
{! examples/hardware/custom.py !}
```

Depending on your hardware you may need to modify a PWM signal, send commands via CAN bus or serial, use Protobuf over Ethernet or something else.
By raising an exception if the real hardware is not available, a simulated robot is instantiated instead.
The robot can be controlled by keyboard or joystick.

## Robot Brain

The [Zauberzeug Robot Brain](https://zauberzeug.com/products/robot-brain) is an industrial-grade controller which combines artificial intelligence with machinery.
It has a built-in ESP32 microcontroller with [Lizard](https://lizard.dev/) installed to do the actual hardware communication in realtime.

Serial communication is used to send and receive messages between the built-in NVidia Jetson and the microcontroller.
You can call `SerialCommunication.is_possible()` to automatically switch between simulation and real hardware.
The module `WheelsHardware` expects a `RobotBrain`, which controls the `SerialCommunication` with the microcontroller.

```python
{! examples/hardware/robot_brain.py !}
```

With `communication.debug_ui()` you can add some helpful UI elements for debugging the serial communication.
Furthermore, with `robot_brain.developer_ui()` you can add UI elements to configure and reboot [Lizard](https://lizard.dev/).

The Lizard configuration for a differential-steering controlled robot with an [ODrive](https://odriverobotics.com/) might look as follows:

```python
{! examples/hardware/startup.liz !}
```
