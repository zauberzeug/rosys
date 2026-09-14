import multiprocessing

# spawn, not fork (which is broken for Python), regardless of the global start method (#19)
SPAWN_CONTEXT = multiprocessing.get_context('spawn')
