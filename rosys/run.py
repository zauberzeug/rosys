import asyncio
import logging
import os
import shlex
import signal
import subprocess
import uuid
from collections.abc import Callable, Coroutine, Generator
from contextlib import contextmanager
from dataclasses import dataclass
from functools import partial, wraps
from inspect import signature
from pathlib import Path
from typing import Any, ParamSpec, TypeVar

from nicegui import run

from . import core
from .helpers import is_stopping

P = ParamSpec('P')
R = TypeVar('R')

running_cpu_bound_processes: list[str] = []  # NOTE is used in tests to advance time slower until computation is done
running_sh_processes: list[subprocess.Popen] = []
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable[P, R], *args: P.args, **kwargs: P.kwargs) -> R | None:
    if is_stopping():
        return None
    try:
        return await run.io_bound(callback, *args, **kwargs)
    except RuntimeError as e:
        if 'cannot schedule new futures after shutdown' not in str(e):
            raise
    except asyncio.exceptions.CancelledError:
        pass
    return None


def awaitable(func: Callable) -> Callable:
    """decorator to wrap a normal function into an asyncio coroutine"""
    @wraps(func)
    async def inner(*args, **kwargs):
        return await io_bound(func, *args, **kwargs)
    return inner


async def cpu_bound(callback: Callable[P, R], *args: P.args, **kwargs: P.kwargs) -> R | None:
    if is_stopping():
        return None
    with cpu():
        try:
            return await run.cpu_bound(callback, *args, **kwargs)
        except RuntimeError as e:
            if 'cannot schedule new futures after shutdown' not in str(e):
                raise
        except asyncio.exceptions.CancelledError:
            pass
    return None


@contextmanager
def cpu() -> Generator[None, None, None]:
    id_ = str(uuid.uuid4())
    running_cpu_bound_processes.append(id_)
    try:
        yield
    finally:
        running_cpu_bound_processes.remove(id_)


async def sh(command: list[str] | str, *,
             timeout: float | None = 1,
             shell: bool = False,
             working_dir: Path | None = None) -> str:
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
            cmd_list = ['timeout', '--signal=SIGTERM', str(timeout), *cmd_list]
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
            log.exception('failed to run command "%s"', cmd)
            if 'proc' in locals() and proc:
                _kill(proc)
            return ''

    if is_stopping():
        return ''
    try:
        async def popen_coro() -> str:
            return await io_bound(popen) or ''
        return await asyncio.wait_for(popen_coro(), timeout)
    except TimeoutError:
        log.warning('Command "%s" timed out after %s seconds.', command, timeout)
        return ''


def _kill(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        log.info('sent SIGTERM to %s', proc.pid)
        try:
            proc.wait(timeout=5)  # wait for 5 seconds
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)  # force kill if process didn't terminate
            log.info('sent SIGKILL to %s', proc.pid)
            proc.wait()  # ensure the process is reaped
    except ProcessLookupError:
        pass
    except Exception:
        log.exception('Failed to kill and/or wait for process %s', proc.pid)


def tear_down() -> None:
    log.info('teardown shell processes...')
    for process in running_sh_processes:
        _kill(process)
    running_sh_processes.clear()
    log.info('teardown complete.')


@dataclass(slots=True, kw_only=True, frozen=True)
class OnFailedArguments:
    attempt: int
    max_attempts: int


async def wait_for(func: Callable[..., Coroutine[Any, Any, R]], timeout: float) -> R:
    """Call a function with a timeout.

    :param func: A function to call
    :param timeout: Maximum time in seconds to wait for the function to return
    :return: Result of the called function
    :raises TimeoutError: If the function does not return within the given time period
    """
    assert timeout > 0, 'timeout must be greater than 0'

    timeout_task = asyncio.create_task(core.sleep(timeout))
    func_task: asyncio.Task[R] = asyncio.create_task(func())
    done, pending = await asyncio.wait([func_task, timeout_task], return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
    if timeout_task in done and func_task in pending:
        raise TimeoutError()
    assert func_task in done
    return func_task.result()


async def retry(func: Callable, *,
                max_attempts: int = 3,
                max_timeout: float | None = None,
                on_failed: Callable | None = None) -> Any:
    """Call a function repeatedly until it succeeds or reaches the maximum number of attempts.

    :param func: A function to retry
    :param max_attempts: Maximum number of attempts
    :param max_timeout: Optional maximum time in seconds to wait per attempt
    :param on_failed: Optional callback to execute after each failed attempt (optional argument of type ``OnFailedArguments``)
    :return: Result of the called function
    :raises RuntimeError: If all attempts fail
    """
    for attempt in range(max_attempts):
        try:
            if max_timeout is not None:
                return await wait_for(func, max_timeout)
            else:
                return await func()
        except Exception:
            if on_failed is None:
                continue
            if signature(on_failed).parameters:
                result = on_failed(OnFailedArguments(attempt=attempt, max_attempts=max_attempts))
            else:
                result = on_failed()
            if asyncio.iscoroutinefunction(on_failed):
                await result
    func_name = func.func.__name__ if isinstance(func, partial) else func.__name__
    raise RuntimeError(f'Running {func_name} failed.')
