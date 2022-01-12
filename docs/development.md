# Development

## Logging

RoSys uses the Python logging package with namespaced loggers.
For example, the steerer [actor](architecture/actors.md) writes its logs as `rosys.steerer`.
This can be used for fine-granular control of what should show on the console.
As a general starting point we suggest reading the [Python Logging HOWTO](https://docs.python.org/3/howto/logging.html).
In the following examples we use Python's logging `dictConfig` for configuration, because it provides the most flexibility while having all configuration in one place.

### Show Info Messages

To only print RoSys messages at the info level to the console we can use a configuration like this:

```python hl_lines="32-36"
{!src/logging_config.py !}
```

As you move the [joystick](features/manual_steering.md#joystick) `rosys.steerer`, messages will appear on the console:

```
2022-01-11 06:53:21 - INFO - start steering
2022-01-11 06:53:22 - INFO - stop steering
2022-01-11 06:53:23 - INFO - start steering
2022-01-11 06:53:23 - INFO - stop steering
```

### Adding Loggers

You can easily add more loggers.
For example, to see debug messages of the event system you can add

```python hl_lines="1 3"
'rosys.event': {
    'handlers': ['console'],
    'level': 'DEBUG',
    'propagate': False,
},
```

Most of the time we turn off log propagation to ensure the configuration we defined ourselves is really used.

### Logging to File

Sometimes it is helpful to write intensive logging into a file and only show some messages on the console.
For this you can add a file `handler`:

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
    Therefore logging should always be stored outside of your project's source directory.

### Formatting

It is quite useful to see from which file and line number a log entry was triggered.
To keep the log lines from getting too long, you can create a log filter which computes the relative path:

```python hl_lines="8 14"
{!src/log_formatting.py [ln:10-28] !}
```

Now you need to register the filter and apply it in the handler.
Then you can change the format for the formatter:

```python hl_lines="3 9 15"
{!src/log_formatting.py [ln:33-52] !}
```

Log output then looks like this:

```
2022-01-11 06:51:00.319 [DEBUG] rosys/runtime.py:78: startup completed
```

## Profiling

You can add a profiler button to your UI:

```python
from rosys.ui.profiler import create_profiler

...
create_profiler(ui)
```

When the button is pressed, the profiler [yappi](https://github.com/sumerc/yappi) will start recording data.
When stopped, you will see its output on the console.

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
You can call `rosys.ui.objgraph_page()`, which will add the route `/objgraph` to your app.

## Continuous Build

We run our continuous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

We publish releases by creating a new version on GitHub and describe the changes.
A GitHub Action then performs the following steps:

- If the pytests are successful, a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
- A multi-arch docker image is built and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).
