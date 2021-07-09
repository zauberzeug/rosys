# ROS Example

This example shows how to start ROS on the Zauberzeug Robot Brain.
For simplicity we use NiceGUI to present a website with control elements:

<img src="https://raw.githubusercontent.com/zauberzeug/rosys/main/examples/ros/screenshot.png" width="400">

The build in NVIDIA Jetson is running a rather old Ubuntu LTS 18.04.
Therefore it's best to start ROS in a docker container.
We created a small helper script `docker.sh` for simplicity. Run

```bash
./docker.sh build
./docker.sh run
```

to build and start the image in a new container.
You can then also call `./docker.sh attach` to open a new bash session within the running container.

## ROS Nodes

The example constists of two ROS nodes `esp` and `ui`.
The first handles the communication with the ESP32 within the Robot Brain for sending speed commands to the wheels.
The latter is just a demonstration of a seperate node which generates Twist messages through a web interface with NiceGUI. You can replace this with any other node as needed.
