# Actors

## Invocation

### Continuos Invocation

The runtime will invoke the actor's `step` method in the interval defined in the class variable `interval`.

### Delayed Execution

If you want to delay the execution, you should invoke `await rosys.sleep(seconds: float)`.
Using `time.sleep` would result in blocking the whole runtime and `await asyncio.sleep` would delay execution of tests.

## Threading and Multiprocessing

Not every piece of code is already using asyncio.
The actor class provides convenience functions for IO and CPU bound work.

### IO Bound

If you need to read from an external device or use a non-async HTTP library like `requests`, you should wrap the code in a coroutine and await it with `await rosys.run.io_bound(...)`.

### CPU Bound

If you need to do some heavy computation, you should wrap the code in a coroutine and await it with `await rosys.run.cpu_bound(...)`.

## Notifications

Actors can notify the user through `self.notify('message to the user')`.
When using [the default UI](user_interface.md), the notifications will show as snackbar messages.
The history of notifications is stored in the [world](world.md#notifications).
