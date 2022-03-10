#!/usr/bin/env python3
from doctest import OutputChecker
from io import StringIO
import subprocess
import sys
from icecream import ic
import time
import sh


def fail(output, errcode):
    print(f' failed with error code {errcode}: {output}', flush=True)
    sys.exit(errcode)


def check(path: str):
    print(path, end='', flush=True)
    buf = StringIO()
    script = sh.python3(path, _bg=True, _out=buf, _err=buf)
    time.sleep(5)
    output = buf.getvalue()
    if 'Traceback' in output:
        fail(output, 2)
    if 'Error' in output:
        fail(output, 3)

    if 'JustPy ready to go' in output:
        print(f' is ok', flush=True)
    else:
        fail(output, 4)
    try:
        script.terminate()
    except:
        pass


if __name__ == '__main__':
    check('../main.py')
