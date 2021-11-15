# Actors

## Invocation

### Continuos Invocation

The runtime will invoke the actor's `step` method in the interval defined in the class variable `interval`.

### Delayed Execution

If an actor wants to delay execution it should invoke `await self.sleep(delay_in_ms: float)`.
Using `time.sleep` would result in blocking the whole runtime and `await asyncio.sleep` would delay execution of tests.

### Nothing To Do

If an actor decides it has nothing to do in its step, it should raise the `NothingToDo` exception.
This makes sure follow-up actors and derived implementations are not executed.

## Threading and Multiprocessing

Not every piece of code is already using asyncio.
The actor class provides convenience functions for IO and CPU bound work.

### IO Bound

If you need to read from an external device or use a non-async HTTP library like `requests`, you should wrap the code in a function and await it with `Actor.run_io_bound(...)`.

### CPU Bound

If you need to do some heavy computation, you should wrap the code in a function and await it with `Actor.run_cpu_bound(...)`.

## Notifications

Actors can notify the user through `self.notify('message to the user')`.
When using [NiceGUI](user_interface.md), the notifications will show as snackbar messages.
The history of notifications is stored in the [world](world.md#notifications).
