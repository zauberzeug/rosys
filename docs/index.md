# RoSys

RoSys is an easy to use robot system based on modern web technologies.
The purpose is very simlar to [ROS](https://www.ros.org/).
But RoSys is completely build on existing web technologies and always uses the latest version of Python.

## Features

**All Python**
: Business logic is wired in Python while computation heavy tasks are encapsulated through websockets or bindings

**Shared State**
: All code can access and manipulate the global state -- this does not mean it should. 
Good software design is still neccessary 
But its much easier to do if you not have to perform serialization all the time.

**No Threading**
: Thanks to [asyncio](https://docs.python.org/3/library/asyncio.html) you can execute parallel code without locks and mutex mechanisms.

**Web-UI**
: Most machines need some kind of human interaction.
We made sure it can be operated fully off the grid but can also be proxied through a gateway for remote operation.

**Simulation**
: Robot hardware is often slower than your own computer.
Therefore RoSys supports an simulation mode für rapid development.
To get maximal performance the current implementation does not run a full physics engine.

**Testing**
: You can use pytest to write high-level integration tests.
It is based on the above described simulation mode and accelerates the robots time for super fast execution.

!!! note

    Currently RoSys is mostly tested and developed on the [Zauberzeug Robot Brain](https://www.zauberzeug.com/product-robot-brain.html) which uses [Lizard](https://lizard.dev/) for communication with motors, sensors and other peripherals.
    But the software architecture of RoSys also allows you to write your own actors if you perfer another industrial PC or setup.

## Installation

```bash
sudo apt-get install libcurl4-openssl-dev libssl-dev # required for pycurl
python3 -m pip install rosys
```

## Docker

While the above commands may work for you, it's often easier to run RoSys inside a docker container.
There are some specialities needed to start RoSys in different environments.
To simplify the usage we wrapped this in a script `./docker.sh`.

You can configure the dockerized system by putting these variables into the `/.env` file:

- `ESP_SERIAL=/dev/ttyTHS1`: path to the ESP device; default is /dev/null to be able to start RoSys anywhere
- ...
