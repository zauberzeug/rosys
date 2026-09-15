import multiprocessing

# spawn, not fork, regardless of the global start method
SPAWN_CONTEXT = multiprocessing.get_context('spawn')
