# Development

## Continuous Build

We run our continuous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

To release a new version create a new version on GitHub and describe the changes.
A GitHub Action performs the following steps:
If the pytests are successful, a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
Also a multi-arch docker image is built and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).

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
