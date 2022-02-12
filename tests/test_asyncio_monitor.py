from rosys.actors import AsyncioMonitor


def test_parse_could_not_parse_message():
    result = AsyncioMonitor.parse_warning("ESC[32m2022-02-12 18:00:55.497ESC[0m ESC[1;30m[WARNING]ESC[0m rosys/actors/asyncio_monitor.py:63: ESC[33mcould not parse: ESC[32m2022-02-12 18:00:45.497ESC[0m ESC[1;30m[WARNING]ESC[0m asyncio/base_events.py:1885: ESC[33mExecuting <Task pending name='rosys.actors.AsyncioMonitor' coro=<Runtime.repeat() running at /rosys/rosys/runtime.py:100> wait_for=<Future pending cb=[<TaskWakeupMethWrapper object at 0x7f7f6b5e20>()] created at /usr/local/lib/python3.9/asyncio/base_events.py:424> cb=[_handle_task_result(logger=<Logger rosys...ogger (DEBUG)>, message='Task raised an exception', message_args=())() at /rosys/rosys/task_logger.py:38]> took 0.145 secondsESC[0m")
    assert result is None, '"could not parse" message should be ignored'


def test_parse_task_warning():
    msg = "2022-02-12 18:47:17.964 [WARNING] asyncio/base_events.py:1885: Executing <Task pending name='<bound method CameraGrid.update of <outside_in_tracking.ui.cameras.camera_grid.CameraGrid object at 0x7f80ba1ac0>>' coro=<Timer.__init__.<locals>.loop() running at /usr/local/lib/python3.9/site-packages/nicegui/timer.py:57> wait_for=<Future pending cb=[<TaskWakeupMethWrapper object at 0x7f8090bb20>()] created at /usr/local/lib/python3.9/asyncio/base_events.py:424> cb=[_handle_task_result(logger=<Logger niceg...ger (WARNING)>, message='Task raised an exception', message_args=())() at /usr/local/lib/python3.9/site-packages/nicegui/task_logger.py:41]> took 0.237 seconds"
    result = AsyncioMonitor.parse_warning(msg)
    assert result is not None
    assert result.millis == 237
    assert result.name == '<bound method CameraGrid.update of <outside_in_tracking.ui.cameras.camera_grid.CameraGrid object at 0x7f80ba1ac0>>'


def test_parse_tasks_warning():
    msg = "2022-02-10 06:16:37.192 [WARNING] asyncio/base_events.py:1885: Executing <TimerHandle when=12768.78187203222 _set_result_unless_cancelled(<Future finis...events.py:424>, None) at /opt/homebrew/Cellar/python@3.9/3.9.7_1/Frameworks/Python.framework/Versions/3.9/lib/python3.9/asyncio/futures.py:308 created at /opt/homebrew/Cellar/python@3.9/3.9.7_1/Frameworks/Python.framework/Versions/3.9/lib/python3.9/asyncio/tasks.py:651> took 0.075 seconds"
    result = AsyncioMonitor.parse_warning(msg)
    assert result.millis == 75
    assert result.name == 'TimerHandle'
