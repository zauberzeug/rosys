from rosys.analysis import AsyncioMonitor


def test_parse_task_warning():
    msg = "2022-02-12 18:47:17.964 [WARNING] asyncio/base_events.py:1885: Executing <Task pending name='<bound method CameraGrid.update of <outside_in_tracking.ui.cameras.camera_grid.CameraGrid object at 0x7f80ba1ac0>>' coro=<Timer.__init__.<locals>.loop() running at /usr/local/lib/python3.9/site-packages/nicegui/timer.py:57> wait_for=<Future pending cb=[<TaskWakeupMethWrapper object at 0x7f8090bb20>()] created at /usr/local/lib/python3.9/asyncio/base_events.py:424> cb=[_handle_task_result(logger=<Logger niceg...ger (WARNING)>, message='Task raised an exception', message_args=())() at /usr/local/lib/python3.9/site-packages/nicegui/task_logger.py:41]> took 0.237 seconds"
    result = AsyncioMonitor().parse_async_warning(msg)
    assert result is not None
    assert result.duration == 0.237
    assert result.name == '<bound method CameraGrid.update of <outside_in_tracking.ui.cameras.camera_grid.CameraGrid object at 0x7f80ba1ac0>>'


def test_parse_timer_warning():
    msg = '2022-02-10 06:16:37.192 [WARNING] asyncio/base_events.py:1885: Executing <TimerHandle when=12768.78187203222 _set_result_unless_cancelled(<Future finis...events.py:424>, None) at /opt/homebrew/Cellar/python@3.9/3.9.7_1/Frameworks/Python.framework/Versions/3.9/lib/python3.9/asyncio/futures.py:308 created at /opt/homebrew/Cellar/python@3.9/3.9.7_1/Frameworks/Python.framework/Versions/3.9/lib/python3.9/asyncio/tasks.py:651> took 0.075 seconds'
    result = AsyncioMonitor().parse_async_warning(msg)
    assert result.duration == 0.075
    assert result.name == 'TimerHandle'


def test_parse_coro_from_task_warning():
    msg = "2022-02-13 17:26:01.174 [WARNING] asyncio/base_events.py:1885: Executing <Task pending name='Task-3716' coro=<handle_event() running at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:361> wait_for=<_GatheringFuture pending cb=[<TaskWakeupMethWrapper object at 0x7f64aecca0>()] created at /usr/local/lib/python3.9/asyncio/tasks.py:702> created at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:276> took 0.506 seconds"
    result = AsyncioMonitor().parse_async_warning(msg)
    assert result.duration == 0.506
    assert result.name == 'handle_event()'
    assert result.details == 'running at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:361> wait_for=<_GatheringFuture pending cb=[<TaskWakeupMethWrapper object at 0x7f64aecca0>()] created at /usr/local/lib/python3.9/asyncio/tasks.py:702> created at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:276'
