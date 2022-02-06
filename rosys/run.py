import asyncio
from typing import Callable
import subprocess
from concurrent.futures import ProcessPoolExecutor
import sys

process_pool = ProcessPoolExecutor()
heavy_computation = False


async def io_bound(callback: Callable, *args: any):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, callback, *args)


async def cpu_bound(callback: Callable, *args: any):
    global heavy_computation
    if 'pytest' not in sys.modules:
        try:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(process_pool, callback, *args)
        except RuntimeError as e:
            if 'cannot schedule new futures after shutdown' in str(e):
                pass
            else:
                raise
    else:
        try:
            heavy_computation = True
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(None, callback, *args)
        finally:
            heavy_computation = False


async def sh(command: list[str]) -> str:
    '''executes a shell command
    command: a sequence of program arguments as subprocess.Popen requires
    returns: stdout
    '''

    def run() -> str:
        proc = subprocess.Popen(
            command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT)
        stdout, *_ = proc.communicate()
        return stdout.decode()
    #self.log.debug('executing sh command: ' + ' '.join(command))
    return await io_bound(run)
