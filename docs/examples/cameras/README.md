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

## Automatic Reconnection

Cameras can lose their connection due to network glitches, a bad cable or a power hiccup.
Every camera reconnects on its own: once connected, the underlying device keeps trying to restore its stream every `reconnect_interval` seconds (default 3.0) for as long as the camera stays connected.
`RtspCamera` and `MjpegCamera` re-open their stream, `UsbCamera` re-opens the video device (even if its `/dev/video*` node changed), and `SimulatedCamera` resumes after a simulated drop.
Reconnection runs until the camera is disconnected, so `disconnect()` both stops the retries and tears down the device.
`is_connected` tells whether a camera is streaming right now, while `is_active` tells whether a connection is wanted at all, i.e. whether the camera keeps trying.

`connect()` always creates that device, even when the camera cannot be reached at all — because its address is not known yet, or no `/dev/video*` node exists.
Such a camera reports `is_active` but not `is_connected`, and starts streaming as soon as the address or the video device appears.

Cameras that reject our credentials are the one case where retrying is throttled instead of continuing at `reconnect_interval`:
`RtspCamera` and `MjpegCamera` fall back to one attempt per minute, because some cameras answer repeated failed logins by locking the account.
Such a camera stays active and recovers on its own once it accepts the login again; `device.authorized` reports `False` while it backs off.
A camera's own `username` and `password` are read when its device is created, so use `reconnect()` to retry with changed credentials.

Camera providers therefore do not connect or reconnect cameras themselves.
Their periodic scan only discovers new cameras and hands a changed address (e.g. after a new DHCP lease) to the cameras it already knows, which the running device picks up for its next attempt.
A newly discovered camera still connects on its own, because `connect_after_init` defaults to `True`.
A deliberately disconnected camera stays disconnected, even with `auto_scan` enabled.

## Streaming RTSP Cameras

The following example shows how to stream images from an RTSP camera.

```python
{! examples/cameras/streams.py !}
```
