# Development

## Continuous Build

We run our continous integration with GitHub Actions.
For each commit the pytests are executed.

## Releases

To release a new version create a new version on GitHub and describe the changes.
A GitHub Action performs the following steps:
If the pytests are successful a poetry build and deployment to [pypi](https://pypi.org/project/rosys/) is issued.
Also a multi-arch docker image is build and pushed to [Docker Hub](https://hub.docker.com/r/zauberzeug/rosys).

## Profiling

In the system container run

    kernprof profiling.py

to generate the profiling data.
View it with the interactive tool

    python -m pstats profiling.py.prof
