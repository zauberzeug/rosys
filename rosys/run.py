import asyncio
import logging
import os
import signal
import subprocess
import uuid
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
from typing import Callable

process_pool = ProcessPoolExecutor(max_workers=10)
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


async def sh(command: list[str], timeout: float = 1) -> str:
    '''executes a shell command
    command: a sequence of program arguments as subprocess.Popen requires
    returns: stdout
    '''
    def run() -> str:
        with subprocess.Popen(command, preexec_fn=os.setsid, stdout=asyncio.subprocess.PIPE,
                              stderr=asyncio.subprocess.STDOUT) as proc:
            try:
                stdout, *_ = proc.communicate(timeout=timeout)
            except subprocess.TimeoutExpired:
                log.warning(f'{" ".join(command)} took longer than {timeout} s. Aborting.')
                os.killpg(proc.pid, signal.SIGINT)  # send signal to the process group
                stdout, *_ = proc.communicate()
            return stdout.decode()
    #self.log.debug('executing sh command: ' + ' '.join(command))
    return await io_bound(run)
