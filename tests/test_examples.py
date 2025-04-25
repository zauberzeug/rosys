#!/usr/bin/env python3
import os
import sys
import time
from io import StringIO
from pathlib import Path

import sh


def check(path: Path, *, timeout: float = 15.0) -> bool:
    try:
        # kill process which occupies port 8080
        sh.fuser('-k', '8080/tcp')  # get "fuser" command with "apt install psutils" (it is not available on mac)
    except sh.ErrorReturnCode_1:
        pass  # it is ok to not find any process to kill
    print(path, flush=True)
    buf = StringIO()
    script = sh.python3(path, _bg=True, _bg_exc=False, _out=buf, _err=buf)
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.1)
        output = buf.getvalue()
        if 'NiceGUI ready to go' in output:
            break

    try:
        script.terminate()
        script.wait(1)
    except (ProcessLookupError, sh.SignalException_SIGKILL, sh.TimeoutException):
        pass

    if 'Traceback' in output:
        print(f'  Error ("Traceback" found): {output}', flush=True)
        return False

    if 'Error' in output:
        print(f'  Error ("Error" found): {output}', flush=True)
        return False

    if 'ui.run' in path.read_text() and 'NiceGUI ready to go' not in output:
        print(f'  Error (NiceGUI welcome message missing): {output}', flush=True)
        return False

    print('  Ok', flush=True)
    return True


if __name__ == '__main__':
    ROOT_PATH = Path(__file__).parent.parent
    success = check(ROOT_PATH / 'main.py')

    for filepath in (ROOT_PATH / 'docs').glob('**/*.py'):
        if os.access(filepath, os.X_OK):
            success &= check(filepath)

    if not success:
        sys.exit(1)
