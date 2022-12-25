import os

import psutil
from fastapi import Request
from psutil._common import bytes2human
from starlette.middleware.base import BaseHTTPMiddleware


class MemoryMiddleware(BaseHTTPMiddleware):

    async def dispatch(self, request: Request, call_next):
        mem_before = self.get_process_memory()
        response = await call_next(request)
        if response.headers.get('x-nicegui-content') == 'page':
            mem_after = self.get_process_memory()
            msg = f'GET {request.get("path")} increased memory by {bytes2human(mem_after - mem_before)}'
            print(msg, flush=True)
        return response

    @staticmethod
    def get_process_memory() -> int:
        process = psutil.Process(os.getpid())
        mem_info = process.memory_info()
        return mem_info.rss
