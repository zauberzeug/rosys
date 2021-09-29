# RoSys - Zauberzeug Robot System

Currently RoSys is meant to be run on the Zauberzeug Robot Brain.

## Usage

### Installation

```bash
sudo apt-get install libcurl4-openssl-dev libssl-dev # required for pycurl
python3 -m pip install rosys
```

## Development

### Docker

It's easierst to run RoSys inside a docker container. There are some specialities needed to start the RoSys in different environments. To simplify the usage we wrapped this in a script `./docker.sh`.

You can configure the dockerized system by putting these variables into the `/.env` file:

- ESP_SERIAL=/dev/ttyTHS1: path to the esp devices; default is /dev/null to be able to start RoSys anywhere
- ...

### Remote Development

You can develop quite alot of functionality with a simulated robot on your on computer.
But there comes a time when you want to run your code on a real robot.
Normally you will therefore start the container on the Robot Brain and connect via Wifi to the web interface.
By using VS Code Remote Containers you can continue development as if you are using your own computer.
Unfortunately the Jetson hardware is much much slower than your own machine.
With large code base this can result in long restart times after you change some code (30 sec or more).

By launching the [esp_proxy.py](https://github.com/zauberzeug/rosys/blob/main/controller/esp_proxy.py) on the robot you can keep developing on your computer but sending and receiving hardware commands from the Robot Brain through the network.
When the runtime is initalized it will first try to find the ESP32 of the Robot Brain locally.
If this does not work, it tries to reach the Robot Brain via the local WiFi connection.
Only if this also fails it will fallback on a simulated hardware system.

### Continuous Build

We build with Drone. Run locally with

    drone starlark .drone.star && drone exec --trusted --exclude slack  .drone.yml

### Profiling

In the system container run

    kernprof profiling.py

to generate the profiling data. View them with the interactive tool

    python -m pstats profiling.py.prof

###
