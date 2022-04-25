# Cameras

RoSys provides instant camera access for [object detection](object_detection.md), remote operation and similar use cases.
Any plugged in camera becomes an entry in `world.usb_cameras` containing recorded images and configuration parameters like exposure.

## Setup

Camera devices are discovered through video4linux (v4l) and accessed with openCV.
Therefore the program `v4l2ctl` and openCV (including python bindings) must be available.
We recommend to use the [RoSys docker image](https://hub.docker.com/r/zauberzeug/rosys) which provides the full required software stack.
Make sure the container can access the USB devices by starting it with `privileged` or explicitly passing the specific `devices`.

## Show Captured Images

Through the use of `rosys.ui` (see [User Interface](../architecture/user_interface.md)) you can launch a website which shows the latest captured images from each camera:

```python hl_lines="13-22"
{!src/show_captured_images.py [ln:1-4] !}
{!src/show_captured_images.py [ln:5-22] !}
{!src/show_captured_images.py [ln:28-29] !}
```

The `ui.timer` regularly updates the source property of the `ui.image`.
The cameras `latest_image_uri` property provides the uri to the latest captured image.

## Simulated Cameras

The above code works out-of-the-box if your camera can be discoverd with `v4l2ctl`.
When you have no camera at hand or develop parts of your robot on a Mac or Windows system, you can simply add a simulated camera:

```python hl_lines="8-15"
{!src/show_captured_images.py [ln:5] !}
...
{!src/show_captured_images.py [ln:23-27] !}
```

## Remote Operation

A fairly often required use case is the remote operation of a robot.
In a simple use case you may only need to visualize one camera and have some steering controls.
Here we use the `event.Id.NEW_CAMERA` to only display the first camera:

```python hl_lines="16 20"
{!src/remote_operation.py [ln:1-21]!}
```

By adding `joystick` and `keyboard_control` the robot is ready to go for remote operation:

```python hl_lines="4 5"
{!src/remote_operation.py [ln:22-29]!}
```
