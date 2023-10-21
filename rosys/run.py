import asyncio
import logging
import os
import shlex
import signal
import subprocess
import uuid
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from contextlib import contextmanager
from functools import wraps
from pathlib import Path
from typing import Callable, Generator, Optional, cast

from nicegui import run
from psutil import Popen

from .helpers import is_stopping, is_test

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
running_cpu_bound_processes: list[str] = []  # NOTE is used in tests to advance time slower until computation is done
running_sh_processes: list[Popen] = []
log = logging.getLogger('rosys.run')


io_bound = run.io_bound
cpu_bound = run.cpu_bound


def awaitable(func: Callable) -> Callable:
    """decorator to wrap a normal function into an asyncio coroutine"""
    @wraps(func)
    async def inner(*args, **kwargs):
        return await io_bound(func, *args, **kwargs)
    return inner


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
            cmd_list = ['timeout', '--signal=SIGKILL', str(timeout)] + cmd_list
        cmd = ' '.join(cmd_list) if shell else cmd_list
        # log.info(f'running sh: "{cmd}"')
        try:
            with subprocess.Popen(  # pylint: disable=consider-using-with
                cmd,
                cwd=working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=shell,
                start_new_session=True,
            ) as proc:
                running_sh_processes.append(cast(Popen, proc))
                stdout, stderr = proc.communicate()
                proc.stdout.close()
                proc.stderr.close()
                _kill(cast(Popen, proc))
                running_sh_processes.remove(cast(Popen, proc))
                return stdout.decode('utf-8') if proc.returncode == 0 else stderr.decode('utf-8')
        except Exception:
            log.exception(f'failed to run command "{cmd}"')
            if 'proc' in locals() and proc:
                _kill(cast(Popen, proc))
            return ''

    if is_stopping():
        return ''
    try:
        return await asyncio.wait_for(io_bound(popen), timeout)
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
    except Exception as e:
        log.error(f"Failed to kill and/or wait for process {proc.pid}: {e}")


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
