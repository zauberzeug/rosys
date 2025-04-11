# Installation

## On Your Computer

```bash
python3 -m pip install rosys
```

See [Getting Started](getting_started/README.md) for what to do next.

Please note that RoSys has been developed for Unix-like systems.
While it may partially work on Windows, we do not officially support it.

## On The Robot

While the above-mentioned installation command works perfectly well in local environments, on a robot it is often easier to run RoSys inside a Docker container.
If you already have a `main.py`, it can be as simple as running

```
docker run -it --rm -v `pwd`:/app zauberzeug/rosys
```

from the same directory.
See [Pushing Code to Robot](development/README.md#pushing-code-to-robot) on how to get your project onto the remote system.

More complex Docker setups benefit from a compose file.
There are also some specialties needed to start RoSys in different environments (Mac, Linux, NVidia Jetson, ...).
To simplify the usage we suggest to use a script called [`./docker.sh`](https://github.com/zauberzeug/rosys/blob/main/docker.sh) which you can also copy and adapt in your own project.
Have a look at the [project examples](https://github.com/zauberzeug/rosys/tree/main/examples) to see how a setup of your own repository may look like.
