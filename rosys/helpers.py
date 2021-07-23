import inspect
import time
import traceback
import sys
import numpy as np


def measure(*, reset: bool = False, ms: bool = False):

    global t
    if 't' in globals() and not reset:
        dt = time.time() - t
        line = inspect.stack()[1][0].f_lineno
        output = f'{dt * 1000:7.3f} ms' if ms else f'{dt:7.3f} s'
        print(f'{inspect.stack()[1].filename}:{line}', output, flush=True)
    if reset:
        print('------------', flush=True)
    t = time.time()


def print_stacktrace():

    print("-" * 60)
    traceback.print_exc(file=sys.stdout)
    print("-" * 60, flush=True)


def euler_to_rotation_matrix(omega: float, phi: float, kappa: float) -> list[list[float]]:

    Rx = np.array([[1, 0, 0], [0, np.cos(omega), -np.sin(omega)], [0, np.sin(omega), np.cos(omega)]])
    Ry = np.array([[np.cos(phi), 0, np.sin(phi)], [0, 1, 0], [-np.sin(phi), 0, np.cos(phi)]])
    Rz = np.array([[np.cos(kappa), -np.sin(kappa), 0], [np.sin(kappa), np.cos(kappa), 0], [0, 0, 1]])
    return (Rz @ Ry @ Rx).tolist()
