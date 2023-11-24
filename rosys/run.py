import asyncio
import logging
import os
import shlex
import signal
import subprocess
import uuid
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from contextlib import contextmanager
from functools import partial, wraps
from pathlib import Path
from typing import Any, Callable, Generator, Optional

from psutil import Popen

from .helpers import is_stopping, is_test

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
running_cpu_bound_processes: list[str] = []  # NOTE is used in tests to advance time slower until computation is done
running_sh_processes: list[Popen] = []
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable, *args: Any, **kwargs: Any):
    if is_stopping():
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
    """decorator to wrap a normal function into an asyncio coroutine"""
    @wraps(func)
    async def inner(*args, **kwargs):
        return await io_bound(func, *args, **kwargs)
    return inner


async def cpu_bound(callback: Callable, *args: Any):
    if is_stopping():
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
    id_ = str(uuid.uuid4())
    running_cpu_bound_processes.append(id_)
    try:
        yield
    finally:
        running_cpu_bound_processes.remove(id_)


async def sh(command: list[str] | str, *,
             timeout: Optional[float] = 1,
             shell: bool = False,
             working_dir: Optional[Path] = None) -> str:
    """executes a shell command

    Args:
        command: a sequence of program arguments as subprocess.Popen requires or full string
        shell: whether a subshell should be launched (default is `False`, for speed, use `True` if you need file globbing or other features)

    Returns:
        stdout
    """
    def popen() -> str:
        cmd_list = command if isinstance(command, list) else shlex.split(command)
        if timeout is not None:
            cmd_list = ['timeout', '--signal=SIGTERM', str(timeout)] + cmd_list
        cmd = ' '.join(cmd_list) if shell else cmd_list
        try:
            with subprocess.Popen(
                cmd,
                cwd=working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=shell,
                start_new_session=True,
            ) as proc:
                running_sh_processes.append(proc)
                stdout, stderr = proc.communicate()
                assert proc.stdout is not None
                assert proc.stderr is not None
                proc.stdout.close()
                proc.stderr.close()
                _kill(proc)
                running_sh_processes.remove(proc)
                return stdout.decode('utf-8') if proc.returncode == 0 else stderr.decode('utf-8')
        except Exception:
            log.exception(f'failed to run command "{cmd}"')
            if 'proc' in locals() and proc:
                _kill(proc)
            return ''

    if is_stopping():
        return ''
    try:
        return await asyncio.wait_for(io_bound(popen), timeout) or ''
    except asyncio.TimeoutError:
        log.warning(f'Command "{command}" timed out after {timeout} seconds.')
        return ''


def _kill(proc: Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        log.info(f'sent SIGTERM to {proc.pid}')
        try:
            proc.wait(timeout=5)  # wait for 5 seconds
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)  # force kill if process didn't terminate
            log.info(f'sent SIGKILL to {proc.pid}')
            proc.wait()  # ensure the process is reaped
    except ProcessLookupError:
        pass
    except Exception:
        log.exception(f'Failed to kill and/or wait for process {proc.pid}')


def tear_down() -> None:
    # stopping process as described in https://www.cloudcity.io/blog/2019/02/27/things-i-wish-they-told-me-about-multiprocessing-in-python/
    log.info('teardown thread_pool')
    thread_pool.shutdown(wait=False, cancel_futures=True)
    for process in running_sh_processes:
        _kill(process)
    running_sh_processes.clear()
    if not is_test():
        log.info('teardown process_pool')
        for process in process_pool._processes.values():  # pylint: disable=protected-access
            process.kill()
        process_pool.shutdown(wait=True, cancel_futures=True)
    log.info('teardown complete')
