import gc
import logging
import os
import tracemalloc

import psutil
from fastapi import Request
from psutil._common import bytes2human
from starlette.applications import ASGIApp
from starlette.middleware.base import BaseHTTPMiddleware

from .. import rosys, run

log = logging.getLogger('rosys.analysis.memory')


class MemoryMiddleware(BaseHTTPMiddleware):

    def __init__(self, app: ASGIApp) -> None:
        super().__init__(app)
        self.last_mem: int | None = None

    async def dispatch(self, request: Request, call_next):
        mem = get_process_memory()
        response = await call_next(request)
        if response.headers.get('x-nicegui-content') == 'page' and self.last_mem is not None:
            log.info('GET %s increased memory by %s and is now %s',
                     request.get('path'), bytes2human(mem - self.last_mem), bytes2human(mem))
        self.last_mem = mem
        return response


def get_process_memory() -> int:
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss


def get_humanreadable_process_memory() -> str:
    return bytes2human(get_process_memory())


def compare_tracemalloc_snapshots(snapshot, prev_snapshot):
    stats = snapshot.compare_to(prev_snapshot, 'traceback')
    for stat in stats[:10]:
        trace = '\n'.join(stat.traceback.format())
        usage = f'{bytes2human(stat.size_diff)} new, {bytes2human(stat.size)} total; {stat.count_diff} new memory blocks, {stat.count} total'
        log.info('%s"\n"%s', trace, usage)


class _MemoryObserver:

    def __init__(self, with_tracemalloc: bool) -> None:
        self.with_tracemalloc = with_tracemalloc
        self.prev_memory: int = 0
        self.prev_snapshot: tracemalloc.Snapshot | None = None

    async def stats(self) -> None:
        gc.collect()
        growth = get_process_memory() - self.prev_memory
        log.info("memory growth: %s, now it's %s", bytes2human(growth), get_humanreadable_process_memory())
        self.prev_memory = get_process_memory()
        if self.with_tracemalloc:
            snapshot = tracemalloc.take_snapshot()
            if growth > 4 * 1e-6 and self.prev_snapshot is not None:
                await run.cpu_bound(compare_tracemalloc_snapshots, snapshot, self.prev_snapshot)
            self.prev_snapshot = snapshot


_observers: list[_MemoryObserver] = []  # NOTE: keeps the observers alive so their repetitions run until shutdown


def observe_memory_growth(with_tracemalloc: bool = False) -> None:
    log.info('Observing memory growth')
    if with_tracemalloc:
        tracemalloc.start(10)
    observer = _MemoryObserver(with_tracemalloc)
    _observers.append(observer)
    rosys.on_repeat(observer.stats, 60.0)
