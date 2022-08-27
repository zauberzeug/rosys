# Cameras

RoSys provides instant camera access for object detection, remote operation and similar use cases.

## Setup

USB camera devices are discovered through video4linux (v4l) and accessed with openCV.
Therefore the program `v4l2ctl` and openCV (including python bindings) must be available.
We recommend to use the [RoSys docker image](https://hub.docker.com/r/zauberzeug/rosys) which provides the full required software stack.
Make sure the container can access the USB devices by starting it with `--privileged` or explicitly passing the specific `--device`s.

## Show Captured Images

Using `rosys.ui` you can show the latest captured images from each camera:

```python
{!src/example_cameras_images.py !}
```

The `ui.timer` regularly updates the source property of the `ui.image`.
The cameras `latest_image_uri` property provides the URI to the latest captured image.

This example uses a `UsbCameraProviderSimulation` with a single simulated test camera.
But you can replace the provider with a `UsbCameraProviderHardware`.

## Remote Operation

A fairly often required use case on real mobile robots is the remote operation.
In a simple use case you may only need to visualize one camera and have some steering controls.
Here we use the `NEW_CAMERA` event to display the first camera to control real [Hardware](hardware.md):

```python
{!src/example_cameras_remote.py !}
```

By adding a `Joystick` and `KeyboardControl` the robot is ready to go for remote operation.
