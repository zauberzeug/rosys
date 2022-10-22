import asyncio
import logging
import os
import shlex
import signal
import subprocess
import uuid
from asyncio.subprocess import Process
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from contextlib import contextmanager
from functools import partial, wraps
from typing import Callable, Generator, Optional

from psutil import Popen

import rosys

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
running_cpu_bound_processes: list[int] = []  # NOTE is used in tests to advance time slower until computation is done
running_sh_processes: list[Process] = []
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable, *args: any, **kwargs: any):
    if rosys.is_stopping():
        return
    loop = asyncio.get_running_loop()
    try:
        return await loop.run_in_executor(thread_pool, partial(callback, *args, **kwargs))
    except RuntimeError as e:
        if 'cannot schedule new futures after shutdown' not in str(e):
            raise
    except asyncio.exceptions.CancelledError:
        pass


def awaitable(func: Callable) -> Callable:
    '''decorator to wrap a normal function into an asyncio coroutine'''
    @wraps(func)
    async def inner(*args, **kwargs):
        return await io_bound(func, *args, **kwargs)
    return inner


async def cpu_bound(callback: Callable, *args: any):
    if rosys.is_stopping():
        return
    with cpu():
        try:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(process_pool, callback, *args)
        except BrokenProcessPool:
            pass
        except RuntimeError as e:
            if 'cannot schedule new futures after shutdown' not in str(e):
                raise
        except asyncio.exceptions.CancelledError:
            pass


@contextmanager
def cpu() -> Generator[None, None, None]:
    id = str(uuid.uuid4())
    running_cpu_bound_processes.append(id)
    try:
        yield
    finally:
        running_cpu_bound_processes.remove(id)


async def sh(command: list[str] | str, timeout: Optional[float] = 1, shell: bool = False, working_dir: Optional[str] = None) -> str:
    '''executes a shell command

    Args:
        command: a sequence of program arguments as subprocess.Popen requires or full string
        shell: whether a subshell should be launched (default is `False`, for speed, use `True` if you need file globbing or other features)

    Returns:
        stdout
    '''
    def popen() -> str:
        if shell:  # convert to string
            cmd = ' '.join(command) if isinstance(command, list) else command
        else:  # convert to list
            cmd = shlex.split(command) if isinstance(command, str) else command
        if timeout is not None:
            if shell:
                cmd = f'timeout {timeout} {cmd}'
            else:
                cmd = ['timeout', str(timeout)] + cmd
        # log.info(f'running sh: "{cmd}"')
        proc = subprocess.Popen(
            cmd,
            cwd=working_dir,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            shell=shell,
            start_new_session=True,
        )
        running_sh_processes.append(proc)
        stdout, *_ = proc.communicate()
        _kill(proc)
        running_sh_processes.remove(proc)
        return stdout.decode('utf-8')
    if rosys.is_stopping():
        return
    return await io_bound(popen)


def _kill(proc: Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        log.info(f'killed {proc.pid}')
    except ProcessLookupError:
        pass


def tear_down() -> None:
    # stopping process as described in https://www.cloudcity.io/blog/2019/02/27/things-i-wish-they-told-me-about-multiprocessing-in-python/
    log.info('teardown thread_pool')
    thread_pool.shutdown(wait=False, cancel_futures=True)
    [_kill(p) for p in running_sh_processes]
    running_sh_processes.clear()
    if not rosys.is_test:
        log.info('teardown process_pool')
        [p.kill() for p in process_pool._processes.values()]
        process_pool.shutdown(wait=True, cancel_futures=True)
    log.info('teardown complete')
