# Cameras

RoSys provides instant camera access for object detection, remote operation and similar use cases.

## Setup

USB camera devices are discovered through video4linux (v4l) and accessed with openCV.
Therefore the program `v4l2ctl` and openCV (including python bindings) must be available.
We recommend to use the [RoSys Docker image](https://hub.docker.com/r/zauberzeug/rosys) which provides the full required software stack.
Make sure the container can access the USB devices by starting it with `--privileged` or explicitly passing the specific `--device`s.

## Show Captured Images

Using `rosys.ui` you can show the latest captured images from each camera:

```python
{! examples/cameras/images.py !}
```

The `ui.timer` regularly updates the source property of the `ui.image`.
The cameras `latest_image_uri` property provides the URI to the latest captured image.

This example uses a `SimulatedCamera` for demonstration.
You can directly replace the camera with a `UsbCamera` or `RtspCamera` if you know their ids
or use their respective providers to discover them automatically.

## Remote Operation

A fairly often required use case on real mobile robots is the remote operation.
In a simple use case you may only need to visualize one camera and have some steering controls.
Here we use the `NEW_CAMERA` event to display the first camera to control real [Hardware](../hardware/README.md):

```python
{! examples/cameras/remote.py !}
```

By adding a `Joystick` and `KeyboardControl` the robot is ready to go for remote operation.

## Controlling the Camera

The following example creates a web interface for controlling multiple camera types.
It displays cameras in a grid, showing their live feeds along with controls to connect/disconnect and adjust settings like FPS, quality, exposure, and color.
The demo supports RTSP, MJPEG, USB, and simulated cameras.
It automatically updates every 0.1 seconds to detect and display new cameras, and initializes with one simulated camera.

```python
{! examples/cameras/control.py !}
```

## Streaming RTSP Cameras

The following example shows how to stream images from an RTSP camera.

```python
{! examples/cameras/streams.py !}
```
