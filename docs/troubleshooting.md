# Troubleshooting

## Asyncio Warning

While running RoSys you may see warnings similar to this one:

```
2021-10-31 15:08:04.040 [WARNING] asyncio: Executing <Task pending name='Task-255' coro=<handle_event() running at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:344> wait_for=<_GatheringFuture pending cb=[<TaskWakeupMethWrapper object at 0x7f7001f8e0>()] created at /usr/local/lib/python3.9/asyncio/tasks.py:705> created at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:261> took 0.238 seconds
```

This means some coroutine is clogging the event loop for too long.
In the above example it is a whopping 238 ms in which no other actor can do anything.
This is an eternity when machine communication is expected to happen about every 10 ms.
The warning also provides a (not so readable) hint where the time is consumed.

The example above is one of the more frequent scenarios.
It means some code inside a user interaction event handler (e.g. `handle_event()` in `justpy.py`) is blocking.
Try to figure out which UI event code is responsible by commenting out parts of your logic and try to reproduce the warning systematically.

### macOS Cairo Library Issues

Install system dependencies using Homebrew (Cairo is a 2D graphics library with support for multiple output devices):

```bash
brew install cairo pkg-config
```

If you encounter issues with Python not finding the Cairo library, you may need to set the library path:

```bash
export DYLD_LIBRARY_PATH="/opt/homebrew/lib:$DYLD_LIBRARY_PATH"
```

Or add it to your shell profile (`~/.zshrc` or `~/.bash_profile`).