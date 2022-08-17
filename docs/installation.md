# Installation

## On Your Desktop

```bash
python3 -m pip install rosys
```

See [Getting Started](getting_started.md) for what to do next.

## On The Robot

While the above installation commands work in a well setup environment, it is often easier to run RoSys inside a docker container, especially on Nvidia Jetson devices with their old 18.04 LTS Ubuntu.

### Launching

There are some specialities needed to start RoSys in different environments (Mac, Linux, Jetson, ...).
To simplify the usage we wrapped this in a script called `./docker.sh` which you can also use and adapt in your own project.
Have a look at the [examples](https://github.com/zauberzeug/rosys/tree/main/examples) to see how a setup of your own repository could look like.

### Remote Development

You can develop quite a lot of functionality with a simulated robot on your own computer.
But there comes a time when you want to run your code on a real robot.
Normally you will therefore start the container on the Robot Brain and connect via WiFi to the web interface.
By using VS Code Remote Containers you can continue development as if you are using your own computer.
Unfortunately some robot hardware (for example Nvidia Jetson) is much much slower than your own machine.
With a large code base this can result in long restart times after you change some code (30 seconds or more).

By launching `rosys/hardware/hardware_proxy.py` on the Robot Brain you can keep developing on your computer while being connected to the hardware via WiFi.
When the runtime is initialized, it will first try to find the ESP32 of the Robot Brain locally.
If this does not work, it tries to reach the Robot Brain via the local WiFi connection.
Only if this also fails, it will fallback on a simulated hardware system.
