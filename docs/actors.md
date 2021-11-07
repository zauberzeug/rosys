# Actors

## Continous Invocatation

The Runtime will invoke the Actors `step` method in the interval defined in the class variable `interval`.

### Nothing To Do

If an Actor decides it has nothing to do in it's step, it should raise the `NothingToDo` exception.
This makes sure follow-up actors and derived implementations are not executed.

## Threading and Multiprocessing

Not every piece of code is already using asyncio.
The Actor class provides convincene functions for IO and CPU bound work.

### IO Bound

If you need to read from an external device or use an non-async http library like `requests` you should wrap the code into a function and await it with `Actor.run_io_bound(...)`.

### CPU Bound

If you need to some heavy computation you should wrap the code into a function and await it with `Actor.run_cpu_bound(...)`.
