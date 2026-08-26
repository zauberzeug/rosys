import functools
import ssl

import httpx

DEFAULT_TIMEOUT = 5.0
"""Seconds an idle camera connection is given before it counts as lost."""


@functools.cache
def _ssl_context() -> ssl.SSLContext:
    """Reuse a single SSL context, because building one costs tens of milliseconds of event loop time."""
    return httpx.create_ssl_context()


def new_async_client(*, timeout: float | None = DEFAULT_TIMEOUT, **kwargs) -> httpx.AsyncClient:
    """Create an `httpx.AsyncClient` with a shared SSL context and an explicit timeout.

    Cameras are retried in a loop, so building a fresh SSL context per attempt would stall the event
    loop; and a stream that stops sending data must time out rather than park the capture loop forever.
    """
    return httpx.AsyncClient(verify=_ssl_context(), timeout=timeout, **kwargs)
