import functools
import ssl

import httpx

DEFAULT_TIMEOUT = 5.0
"""Seconds without data before a camera connection counts as lost."""


@functools.cache
def _ssl_context() -> ssl.SSLContext:
    """Build the SSL context once: constructing one costs tens of milliseconds of event loop time."""
    return httpx.create_ssl_context()


def new_async_client(*, timeout: float | None = DEFAULT_TIMEOUT, **kwargs) -> httpx.AsyncClient:
    """Create an `httpx.AsyncClient` with a shared SSL context and an explicit timeout.

    The timeout is explicit because a silent stream must end the request rather than wait forever.
    """
    return httpx.AsyncClient(verify=_ssl_context(), timeout=timeout, **kwargs)
