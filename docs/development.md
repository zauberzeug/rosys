# Development

## Logging

RoSys uses the python logging package with namespaced loggers.
For example the steerer [actor](architecture/actors.md) writes it's logs as `rosys.steerer`.
This can be used for fine-granular control of what should show on the console.
As a general starting point we suggest reading the [Python Logging HOWTO](https://docs.python.org/3/howto/logging.html).
In the following examples we use Pythons logging `dictConfig` for configuration because it provides the most flexibility while having all configuration in one place.

### Show Info Messages

To only print rosys messages at the info level on the console we can use a configuration like this:

```python hl_lines="32-36"
{!src/logging_config.py !}
```

As you move the [joystick](features/manual_steering.md#joystick) `rosys.steerer` messages will appear on the console.

### Adding Loggers

You can easily add more loggers.
For example to see debug messages of the event system you can add

```python hl_lines="1 3"
'rosys.event': {
    'handlers': ['console'],
    'level': 'DEBUG',
    'propagate': False,
},
```

Most of the time we turn of log propagation to ensure the configuration we defined our self is really used.

### Logging to File

Sometimes it's helpful to write intensive logging into a file and only show some messages on the console.
For this you can add an file `handler`:

```python hl_lines="8-15"
{!src/logging_to_file.py [ln:19-34] !}
```

Then you can decide for each logger which handlers should be used:

```python hl_lines="3 8 13 18"
{!src/logging_to_file.py [ln:35-56] !}
```

!!! note

    The above file logger writes to `~/.rosys`.
    For development it is very helpful to have [auto-reloading on file change activated](getting_started.md#start).
    Therefore logging should always be stored outside of your projects source directory.

## Profiling

You can add a profiler button to your UI:

```python
from rosys.ui.profiler import create_profiler

...
create_profiler(ui)
```

When the button is clicked, the profiler [yappi](https://github.com/sumerc/yappi) will start recording data.
When stopped you will see its output on the console.

## Memory Leaks

To analyze memory leaks RoSys allows the integration of [pyloot](https://github.com/reallistic/pyloot) as a separate page:

```python hl_lines="6"
from nicegui import ui
import rosys.ui

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)
rosys.ui.pyloot_page()
```

This will add a route `/pyloot` to your app.
The graphs will continuously update and show you which types of object counts are growing.
You can then inspect them.
To analyze cyclic references the [objgraph](https://mg.pov.lt/objgraph/index.html) library can be very helpful.
You can call `rosys.ui.objgraph_page()` which will add the route `/objgraph` to your app.

## Continuous Build

We run our continuous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

We publish releases by creating a new version on GitHub and describe the changes.
A GitHub Action then performs the following steps:

- If the pytests are successful, a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
- A multi-arch docker image is built and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).
