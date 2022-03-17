import asyncio
from contextlib import contextmanager
from typing import Callable
import subprocess
from concurrent.futures import ProcessPoolExecutor
import logging
import uuid


process_pool = ProcessPoolExecutor()
running_processes = []  # NOTE is used in rosys.test.Runtime to advance time slower until computation is done
log = logging.getLogger('rosys.run')


async def io_bound(callback: Callable, *args: any):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, callback, *args)


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


async def sh(command: list[str]) -> str:
    '''executes a shell command
    command: a sequence of program arguments as subprocess.Popen requires
    returns: stdout
    '''
    def run() -> str:
        proc = subprocess.Popen(command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        stdout, *_ = proc.communicate()
        return stdout.decode()
    #self.log.debug('executing sh command: ' + ' '.join(command))
    return await io_bound(run)
