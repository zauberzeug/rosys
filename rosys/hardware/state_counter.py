from collections.abc import Callable

from .. import rosys


class StateCounter:
    """Follows a Lizard state counter, so a command that started and ended between two core messages still shows.

    The counter's value modulo ``states`` is the state machine's state, 0 is idle;
    it wraps at the largest multiple of ``states`` up to 255, as Lizard's ``StateCounter`` does.
    Every command Lizard accepts takes the counter exactly ``states`` steps from the idle before it to its own end,
    and a refused command leaves it unchanged.
    So commands are counted ahead instead of awaited one by one:
    call :meth:`command` right before sending one and pass the base it returns to :meth:`taken` and :meth:`ended`.
    Commands that only stop or hold start no cycle and must not go through :meth:`command`.
    """
    MAX_UNSEEN_COMMANDS = 10
    """Commands sent and not yet seen to end; a counter farther behind means Lizard restarted."""

    def __init__(self, states: int) -> None:
        if not 2 <= states <= 255:
            raise ValueError('a state counter needs between 2 and 255 states')
        self.states = states
        self.period = 255 // states * states
        self.value = 0
        """the counter as last reported by Lizard"""
        self._commands_end: int | None = None

    @property
    def state(self) -> int:
        """The state machine's current state, 0 being idle."""
        return self.value % self.states

    def command(self) -> int:
        """Count a command about to be sent.

        :return: The counter value at which the command begins.
        """
        base = self._next_base()
        self._commands_end = (base + self.states) % self.period
        return base

    def taken(self, base: int) -> bool:
        """Whether Lizard took the command that begins at ``base``."""
        steps = self._steps_past(base)
        return steps is not None and steps > 0

    def ended(self, base: int) -> bool:
        """Whether the command that begins at ``base`` has ended."""
        steps = self._steps_past(base)
        return steps is not None and steps >= self.states

    def reset(self) -> None:
        """Count the next command from the reported counter, e.g. after a refused command, which never shows."""
        self._commands_end = None

    async def wait_taken(self, base: int, timeout: float) -> bool:
        """Wait until Lizard took the command that begins at ``base``, or until ``timeout`` seconds passed."""
        return await self._wait(lambda: self.taken(base), timeout)

    async def wait_ended(self, base: int, timeout: float) -> bool:
        """Wait until the command that begins at ``base`` has ended, or until ``timeout`` seconds passed."""
        return await self._wait(lambda: self.ended(base), timeout)

    def _next_base(self) -> int:
        if self._commands_end is not None and \
                0 < (self._commands_end - self.value) % self.period <= self.MAX_UNSEEN_COMMANDS * self.states:
            return self._commands_end
        return (self.value + -self.value % self.states) % self.period

    def _steps_past(self, base: int) -> int | None:
        steps = (self.value - base) % self.period
        return steps if steps < self.period // 2 else None

    @staticmethod
    async def _wait(condition: Callable[[], bool], timeout: float) -> bool:
        deadline = rosys.time() + timeout
        while not condition():
            if rosys.time() > deadline:
                return False
            await rosys.sleep(0.01)
        return True
