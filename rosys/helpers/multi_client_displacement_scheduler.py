import asyncio
from collections.abc import Coroutine
from enum import StrEnum
from typing import Any, TypeVar

from ..core import time

_T = TypeVar('_T')


class SubmissionState(StrEnum):
    CANCELED = 'canceled'
    WAITING = 'waiting'
    SCHEDULED = 'scheduled'


class SubmissionData:
    def __init__(self, state: SubmissionState):
        self.state = state
        self.can_progress: asyncio.Event = asyncio.Event()


class Client:
    def __init__(self):
        self.last_run_at: float = -float('inf')
        self.waiting_request: SubmissionData | None = None


class MultiClientDisplacementScheduler:
    """A scheduler that controls access to an (implicit) shared resource.

    The shared resource is not managed by the scheduler itself, but assumed
    to be accessed by the coroutine given to `run`. The maximum number of
    concurrent accesses is controlled by the `running_slots` parameter.

    Clients (identified by id) are scheduled fairly, favouring clients with the
    least recent access to the resource. If the resource is equally contended
    by all clients, this results in round-robin scheduling.

    For each client, there is one waiting slot for accesses, meaning that new
    requests (by calls to `run`) displace old ones, similar to the mechanism
    used by the `LazyWorker`.

    *Note*: The implementation is *not* threadsafe, so it should not be used to
    synchronize access from different threads (e.g., by using a multi threaded
    async runtime).
    """

    def __init__(self, running_slots: int = 1):
        """ Initialize the scheduler with a fixed number of running slots.

        :param running_slots: Maximum number coroutines running concurrently.
        """

        self.clients: dict[str, Client] = {}
        self.running_slots_available: int = running_slots

    async def run(self, client_id: str, coro: Coroutine[Any, Any, _T]) -> _T | None:
        """Try to run the given coroutine. A later call with the same
        `client_id` will displace this one. In that case `None` is returned.

        :param client_id: Uniquely identifies the client and thus waiting slot
        :param coro: Coroutine which accesses a shared resource
        :return: None if a later call displaced this one, the return value of
                 the coroutine otherwise
        """

        data = SubmissionData(state=SubmissionState.WAITING)

        # Make sure we have a client for our id
        if client_id not in self.clients:
            self.clients[client_id] = Client()
        client = self.clients[client_id]

        # Displace waiting request by the current (more recent) one
        if client.waiting_request is not None:
            displaced = client.waiting_request
            displaced.state = SubmissionState.CANCELED
            displaced.can_progress.set()
        client.waiting_request = data

        # If there are running slots available, take one immediately
        if self.running_slots_available > 0:
            self.running_slots_available -= 1
            data.state = SubmissionState.SCHEDULED
            client.waiting_request = None

        while True:
            if data.state == SubmissionState.CANCELED:
                coro.close()
                return None

            if data.state == SubmissionState.SCHEDULED:
                client.last_run_at = time()
                try:
                    return await coro
                finally:
                    next_client = min(((k, v) for k, v in self.clients.items() if v.waiting_request is not None),
                                      key=lambda kv: kv[1].last_run_at,
                                      default=None, )

                    if next_client is None:
                        # Make running slot available
                        self.running_slots_available += 1
                    else:
                        # Pass this running slot to next request
                        next_client_id, _ = next_client
                        next_request = self.clients[next_client_id].waiting_request
                        assert next_request is not None
                        next_request.state = SubmissionState.SCHEDULED
                        self.clients[next_client_id].waiting_request = None
                        next_request.can_progress.set()

            await data.can_progress.wait()
            data.can_progress.clear()
