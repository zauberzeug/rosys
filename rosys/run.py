import asyncio
import logging
import uuid
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from contextlib import contextmanager
from typing import Callable

from .helpers import is_test

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
running_processes = []  # NOTE is used in rosys.test.Runtime to advance time slower until computation is done
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
    running_processes.append(id)
    try:
        yield
    finally:
        running_processes.remove(id)


async def sh(command: list[str], timeout: float = 1) -> str:
    '''executes a shell command
    command: a sequence of program arguments as subprocess.Popen requires
    returns: stdout
    '''
    #cmd_str = ' '.join(command)
    #log.info(f'running sh command "{cmd_str}"')
    proc = await asyncio.create_subprocess_exec(
        'timeout',
        str(timeout), *command,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    stdout, *_ = await proc.communicate()
    #log.info(f'done executing "{cmd_str}"')
    return stdout.decode()


def tear_down():
    log.info('teardown thread_pool')
    thread_pool.shutdown(wait=True, cancel_futures=True)
    if not is_test:
        log.info('teardown process_pool')
        process_pool.shutdown(wait=True, cancel_futures=True)
    log.info('teardown complete')
