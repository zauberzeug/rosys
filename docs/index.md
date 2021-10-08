# RoSys

RoSys is an easy to use robot system based on modern web technologies. The purpose is simlar to [ROS](https://www.ros.org/).

## Features

**All Python**
: business logic is wired in Python while computation heavy tasks are encapsulated through websockets or bindings

**Shared State**
: all code can access and manipulate the global state -- this does not mean it should. Good software design is still neccessary but much easier if you do not have to perform serialization all the time.

**No Thrading**
: Thanks to [asyncio](https://docs.python.org/3/library/asyncio.html) parallel code is executed with the guarantee that the state is not modified until reaching the next `await`.

!!! note

    Currently RoSys is mostly tested and developed on the [Zauberzeug Robot Brain](https://www.zauberzeug.com/product-robot-brain.html) which uses [Lizard](https://lizard.dev/) for communication with motors, sensors and similar.
    But the software architecture of RoSys also allows you to write your own actors if you perfer another industrial PC or setup.

## Installation

```bash
sudo apt-get install libcurl4-openssl-dev libssl-dev # required for pycurl
python3 -m pip install rosys
```

## Docker

While the above may work for you it's often easier to run RoSys inside a docker container. There are some specialities needed to start the RoSys in different environments. To simplify the usage we wrapped this in a script `./docker.sh`.

You can configure the dockerized system by putting these variables into the `/.env` file:

- ESP_SERIAL=/dev/ttyTHS1: path to the esp devices; default is /dev/null to be able to start RoSys anywhere
- ...
