# Cameras

RoSys provides instant camera access for [object detection](object_detection.md), remote operation and similar use cases.
Any plugged in camera becomes an entry in `world.cameras` containing recorded frames and configuration parameters like exposure.

## Setup

Camera devices are discovered through video4linux (v4l) and accessed with openCV.
Therefore the program `v4l2ctl` and openCV (including python bindings) must be available.
The simplest way is to use our [RoSys docker image](https://hub.docker.com/r/zauberzeug/rosys) which provides the full required software stack.
Make sure the container can access the usb devices by starting it with `privileged` or explicit passing the `devices`.

## Show Captured Images

Through the use of `rosys.ui` (see [User Interface](user_interface.md)) you can launch a website which shows the latest captured frames from each camera:

```python hl_lines="12-22"
{!src/show_captured_images.py !}
```

A timer updates the source property of the `ui.image` in a given interval.
The cameras `latest_frame_uri` property provides the uri to the latest captured frame.
Thereby the browser is always fetching new images.

## Configure

tbd.
