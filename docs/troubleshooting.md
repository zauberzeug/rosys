# Troubleshooting

## Asyncio Warning

You may see warnings similar to this one

```
2021-10-31 15:08:04.040 [WARNING] asyncio: Executing <Task pending name='Task-255' coro=<handle_event() running at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:344> wait_for=<_GatheringFuture pending cb=[<TaskWakeupMethWrapper object at 0x7f7001f8e0>()] created at /usr/local/lib/python3.9/asyncio/tasks.py:705> created at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:261> took 0.238 seconds
```

while running RoSys.
This means some coroutine is clogging the event loop for too long.
In the above example it's a whopping 238 ms in which no other actor can do anything.
This is an eternaty when machine communication is expected to happen about every 10 ms.
The warning also provides a (not so readable) hint for where the time is consumed.

The example above is one of the more frequent scenarios.
It means some code inside an user interaction event handler (eg. `handle_event()` in `justpy.py`) is blocking.
Try to figure out which ui event code is responsible by commenting out parts of your logic and try to reproduce the warning sytematically.
