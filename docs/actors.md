# Actors

## Continous Invocation

The runtime will invoke the actor's `step` method in the interval defined in the class variable `interval`.

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
