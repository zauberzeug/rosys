# Development

## Remote Development

You can develop quite a lot of functionality with a simulated robot on your own computer.
But there comes a time when you want to run your code on a real robot.
Normally you will therefore start the container on the Robot Brain and connect via Wifi to the web interface.
By using VS Code Remote Containers you can continue development as if you are using your own computer.
Unfortunately robot hardware (for example NVidia Jetson) is much much slower than your own machine.
With a large code base this can result in long restart times after you change some code (30 seconds or more).

By launching [`sudo ./controller/esp_proxy.py`](https://github.com/zauberzeug/rosys/blob/main/controller/esp_proxy.py) on the Robot Brain you can keep developing on your computer while beeing connected to the hardware via WiFi.
When the runtime is initalized it will first try to find the ESP32 of the Robot Brain locally.
If this does not work, it tries to reach the Robot Brain via the local WiFi connection.
Only if this also fails, it will fallback on a simulated hardware system.

## Continuous Build

We build with Drone.
Run it locally with

    drone starlark .drone.star && drone exec --trusted --exclude slack  .drone.yml

## Profiling

In the system container run

    kernprof profiling.py

to generate the profiling data.
View it with the interactive tool

    python -m pstats profiling.py.prof
