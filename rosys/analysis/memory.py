import logging
import os

import psutil
from fastapi import Request
from nicegui import ui
from psutil._common import bytes2human
from starlette.middleware.base import BaseHTTPMiddleware

import rosys

log = logging.getLogger('rosys.analysis.memory')


class MemoryMiddleware(BaseHTTPMiddleware):

    async def dispatch(self, request: Request, call_next):
        mem_before = get_process_memory()
        response = await call_next(request)
        if response.headers.get('x-nicegui-content') == 'page':
            self.mem_before = get_process_memory()
            mem_after = get_process_memory()
            msg = f'GET {request.get("path")} increased memory by {bytes2human(mem_after - mem_before)} and is now {bytes2human(mem_after)}'
            print(msg, flush=True)
        return response


def get_process_memory() -> int:
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss


def get_humanreadable_process_memory() -> int:
    return bytes2human(get_process_memory())


def observe_memory_growth():
    prev_memory = 0

    def stats():
        nonlocal prev_memory
        log.info(
            f'memory growth: {bytes2human(rosys.analysis.memory.get_process_memory() - prev_memory)},'
            f'now its {rosys.analysis.memory.get_humanreadable_process_memory()}'
        )
        prev_memory = rosys.analysis.memory.get_process_memory()

    ui.timer(10.0, stats)
