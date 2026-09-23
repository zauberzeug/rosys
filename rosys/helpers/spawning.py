import multiprocessing

# fork would copy the parent's threads and event loop into the child; spawn starts it clean
SPAWN_CONTEXT = multiprocessing.get_context('spawn')
