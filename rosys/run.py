import asyncio
import logging
import time
import uuid
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from contextlib import contextmanager
from typing import Callable

import sh as sh_module

process_pool = ProcessPoolExecutor()
thread_pool = ThreadPoolExecutor(thread_name_prefix='run.py thread_pool')
running_processes = []  # NOTE is used in rosys.test.Runtime to advance time slower until computation is done
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable, *args: any):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(thread_pool, callback, *args)


async def cpu_bound(callback: Callable, *args: any):
    with cpu():
        try:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(process_pool, callback, *args)
        except RuntimeError as e:
            if 'cannot schedule new futures after shutdown' not in str(e):
                raise


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
    joined_command = ' '.join(command)
    log.debug(f'executing sh command "{joined_command}"')
    cmd = sh_module.Command(command[0])
    proc = cmd(*command[1:], _bg=True)
    t = time.time()
    while proc.is_alive() and time.time() - t < timeout:
        await asyncio.sleep(0.01)
    if proc.is_alive():
        log.warning(f'{joined_command} took longer than {timeout} s. Aborting.')
        proc.terminate()
    result = proc.stdout.decode()
    log.debug(f'completed sh command "{joined_command}", result starts with "{result[:10]}"')
    return result
