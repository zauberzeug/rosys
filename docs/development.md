# Development

## Pushing Code to Robot

To get the code onto the robot you can simply pull your repository.
But this requires you to have login credentials on an external machine.
And editing files must be done on slow hardware compared to development workstations and laptops.
If you use [VS Code Remote Development](https://code.visualstudio.com/docs/remote/remote-overview) or similar to do actual development on these slow systems, everything feels like jelly. Especially if you run powerful extensions like [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance).

That's why we at [Zauberzeug](https://zauberzeug.com) created a small open source tool called [LiveSync](https://github.com/zauberzeug/livesync).
It combines a local filesystem watcher with rsync to copy changes to a (slow) remote target whenever your local code changes. This approach has multiple advantages:

- own choosing of IDE and tooling
- locally able to run tests (or the production code)
- simultaneously continuous deployment of the development code to the target environment (where auto-reload ensures live preview)
- almost no overhead on the (slow) target

## Logging

RoSys uses the Python [logging](https://docs.python.org/3/library/logging.html) package with namespaced loggers.
For example, the steerer module writes its logs as `rosys.steerer`.
This can be used for fine-granular control of what should show on the console.
As a general starting point we suggest reading the [Python Logging HOWTO](https://docs.python.org/3/howto/logging.html).
In the following examples we use Python's logging `dictConfig` for configuration, because it provides the most flexibility while having all configuration in one place.

### Show Info Messages

To only print RoSys messages at the info level to the console we can use a configuration like this:

```python hl_lines="32-36"
{!src/dev_log_config.py !}
```

As you move the joystick, `rosys.steerer` messages will appear on the console:

```
2022-01-11 06:53:21 - INFO - start steering
2022-01-11 06:53:22 - INFO - stop steering
2022-01-11 06:53:23 - INFO - start steering
2022-01-11 06:53:23 - INFO - stop steering
```

### Adding Loggers

You can easily add more loggers.
For example, to see debug messages of the odometer you can add

```python hl_lines="1 3"
'rosys.odometer': {
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
{!src/dev_log_file.py [ln:22-37] !}
```

Then you can decide for each logger which handlers should be used:

```python hl_lines="3 8 13 18"
{!src/dev_log_file.py [ln:38-59] !}
```

!!! note

    The above file logger writes to `~/.rosys`.
    For development it is very helpful to have [auto-reloading on file change activated](getting_started.md#start).
    Therefore logging should always be stored outside of your project's source directory.

### Formatting

It is quite useful to see from which file and line number a log entry was triggered.
To keep the log lines from getting too long, you can create a log filter which computes the relative path:

```python hl_lines="8 14"
{!src/dev_log_formatting.py [ln:10-28] !}
```

You need to register the filter and apply it in the handler.
Then you can change the format for the formatter:

```python hl_lines="2-4 9 17 22"
{!src/dev_log_formatting.py [ln:39-64] !}
```

Log output then looks like this:

```
2022-01-11 06:51:00.319 [DEBUG] rosys/runtime.py:78: startup completed
```

## Profiling

!!! note

    The default RoSys installation via pip does not come with profiling packages.
    To install them, run

    ```bash
    python3 -m pip install rosys[profiling]
    ```

    This does currently not work with Python 3.11 because yappy and line-profiler have not implemented 3.11 yet.

You can add a `profile` decorator to expensive functions and add a profiler button to your UI:

```python hl_lines="7 16"
{!src/dev_profiling.py !}
```

When the button is pressed, the profiler [yappi](https://github.com/sumerc/yappi) will start recording data.
When stopped, you will see its output on the console:

```
Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
     7                                           @profiling.profile
     8                                           def compute() -> None:
     9         3         21.0      7.0      0.0      s = 0
    10   3000003     433138.0      0.1     28.2      for i in range(1_000_000):
    11   3000000    1098975.0      0.4     71.6          s += i**2
    12         3       2151.0    717.0      0.1      ui.notify(s)
```

## Track async function calls

RoSys provides a `@track` decorator that you can put above asynchronous functions that are called as part of automations.
The UI element `track.ui()` will show the stack of functions that are currently awaited.

```python hl_lines="8 13 18 27"
{!src/dev_track.py !}
```

## Continuous Build

We run our continuous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

We publish releases by creating a new version on GitHub and describe the changes.
A GitHub Action then performs the following steps:

- If the pytests are successful, a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
- A multi-arch docker image is built and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).
