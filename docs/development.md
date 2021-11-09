# Development

## Continuous Build

We run our continous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

To release a new version create a new version on GitHub and describe the changes.
A GitHub Action performs the following steps:
If the pytests are successful, a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
Also a multi-arch docker image is built and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).

## Profiling

You can add an profiler button to your UI:

```python
from rosys.ui.profiler import create_profiler

...

create_profiler(ui)
```

If the button is clicked, the profilier [yappi](https://github.com/sumerc/yappi) will start recording data.
When stopped you will see its output on the console.
