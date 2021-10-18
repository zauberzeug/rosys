# Installation

## On Your Desktop

```bash
sudo apt-get install libcurl4-openssl-dev libssl-dev # required for pycurl
python3 -m pip install rosys
```

See [Getting Started](getting_started.md) for what to do next.

## On The Robot

While the above installation commands work in a well setup environment, it is often easier to run RoSys inside a docker container, especially on Nvidia Jetson devices with their old 18.04 LTS Ubuntu.

### Launching

There are some specialities needed to start RoSys in different environments (Mac, Linux, Jetson, ...).
To simplify the usage we wrapped this in a script called `./docker.sh` which you can also use and adapt in your own project.
Have a look at the [examples](https://github.com/zauberzeug/rosys/tree/main/examples) to see how a setup of your own repository could look like.

### Hardware access

You can configure the dockerized system by putting these variables into the `/.env` file:

- `ESP_SERIAL=/dev/ttyTHS1`: path to the ESP device; default is /dev/null to be able to start RoSys anywhere
- ...

### Remote Development

You can develop quite a lot of functionality with a simulated robot on your own computer.
But there comes a time when you want to run your code on a real robot.
Normally you will therefore start the container on the Robot Brain and connect via WiFi to the web interface.
By using VS Code Remote Containers you can continue development as if you are using your own computer.
Unfortunately some robot hardware (for example Nvidia Jetson) is much much slower than your own machine.
With a large code base this can result in long restart times after you change some code (30 seconds or more).

By launching [`sudo ./controller/esp_proxy.py`](https://github.com/zauberzeug/rosys/blob/main/controller/esp_proxy.py) on the Robot Brain you can keep developing on your computer while beeing connected to the hardware via WiFi.
When the runtime is initalized, it will first try to find the ESP32 of the Robot Brain locally.
If this does not work, it tries to reach the Robot Brain via the local WiFi connection.
Only if this also fails, it will fallback on a simulated hardware system.
