import inspect
import time
import traceback
import sys


def measure(reset: bool = False):

    global t
    if 't' in globals() and not reset:
        dt = time.time() - t
        line = inspect.stack()[1][0].f_lineno
        print(f'{inspect.stack()[1].filename}:{line}', "%6.3f s" % (dt), flush=True)
    if reset:
        print('------------ starting new measurements -----------', flush=True)
    t = time.time()


def print_stacktrace():
    print("-"*60)
    traceback.print_exc(file=sys.stdout)
    print("-"*60, flush=True)
