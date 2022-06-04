import asyncio
import logging
import uuid
from asyncio.subprocess import Process
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from contextlib import contextmanager
from typing import Callable, Optional, Union

from .helpers import is_test

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
# NOTE is used in rosys.test.Runtime to advance time slower until computation is done
running_cpu_bound_processes: list[int] = []
running_sh_processes: list[Process] = []
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable, *args: any):
    loop = asyncio.get_running_loop()
    try:
        return await loop.run_in_executor(thread_pool, callback, *args)
    except RuntimeError as e:
        if 'cannot schedule new futures after shutdown' not in str(e):
            raise
    except asyncio.exceptions.CancelledError:
        pass


async def cpu_bound(callback: Callable, *args: any):
    with cpu():
        try:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(process_pool, callback, *args)
        except RuntimeError as e:
            if 'cannot schedule new futures after shutdown' not in str(e):
                raise
        except asyncio.exceptions.CancelledError:
            pass


@contextmanager
def cpu():
    id = str(uuid.uuid4())
    running_cpu_bound_processes.append(id)
    try:
        yield
    finally:
        running_cpu_bound_processes.remove(id)


async def sh(command: Union[list[str], str], timeout: Optional[float] = 1) -> str:
    '''executes a shell command
    command: a sequence of program arguments as subprocess.Popen requires
    returns: stdout
    '''
    cmd_str = command if isinstance(command, str) else ' '.join(command)
    cmd_with_timeout = cmd_str if timeout is None else f'timeout {timeout} {cmd_str}'
    #log.info(f'running sh command "{cmd_str}"')
    try:
        proc = await asyncio.create_subprocess_shell(
            cmd_with_timeout,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        running_sh_processes.append(proc)
    except:
        return 'could not execute "{cmd_str}"'
    else:
        try:
            stdout, *_ = await proc.communicate()
        except asyncio.exceptions.CancelledError:
            return
        except:
            log.exception(f'"{cmd_str}" failed; waiting for process to finish')
            proc.kill()
            return 'could not execute "{cmd_str}"'
        finally:
            running_sh_processes.remove(proc)
    #log.info(f'done executing "{cmd_str}"')
    return stdout.decode()


def tear_down() -> None:
    log.info('teardown thread_pool')
    thread_pool.shutdown(wait=False, cancel_futures=True)
    [p.kill() for p in running_sh_processes]
    if not is_test:
        log.info('teardown process_pool')
        [p.kill() for p in process_pool._processes.values()]
        process_pool.shutdown(wait=True, cancel_futures=True)
    log.info('teardown complete')
