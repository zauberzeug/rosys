import asyncio
import logging
import os
import signal
import subprocess
import uuid
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from contextlib import contextmanager
from typing import Callable

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
    command.insert(0, 'timeout')
    command.insert(1, str(timeout))

    def run() -> str:
        log.info('running sh command: ' + ' '.join(command))
        with subprocess.Popen(
            command,
            preexec_fn=os.setpgrp,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        ) as proc:
            try:
                stdout, *_ = proc.communicate(timeout=timeout)
            except subprocess.TimeoutExpired:
                log.warning(f'{" ".join(command)} took longer than {timeout} s. Aborting.')
                os.killpg(proc.pid, signal.SIGTERM)
                return ''
            log.info('done executing')
            return stdout.decode('utf-8')
    return await io_bound(run)
