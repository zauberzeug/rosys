# RoSys - Zauberzeug Robot System

Currently RoSys is meant to be run on the Zauberzeug Robot Brain

## Starting

There are some specialities needed to start the RoSys in different environments. To simplify the usage we wrapped this in a script `./docker.sh`.

### Configuration

You can configure the system by putting these variables into the `/.env` file:

- ESP_SERIAL=/dev/ttyTHS1: path to the esp devices; default is /dev/null to be able to start RoSys anywhere
- USE_CAMS=true: weather RoSys should use Zauberzeug Smart Cams for tracking

### Under The Hood

- there is no host network on Mac so the conf is overwritten (on real robots it's necccessary to scan for cameras)
- the services will start automatically on jetson when rebooting or crashing to simplify work with real robots

## Continuous Build

We build with Drone. Run locally with

    drone starlark .drone.star && drone exec --trusted --exclude slack  .drone.yml

## Profiling

In the system container run

    kernprof profiling.py

to generate the profiling data. View them with the interactive tool

    python -m pstats profiling.py.prof
