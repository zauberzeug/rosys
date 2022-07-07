#!/usr/bin/env python3
import sys
import time
from io import StringIO

import sh

has_failures = False


def fail(output, errcode):
    global has_failures
    print(f' failed with error code {errcode}: {output}', flush=True)
    has_failures = True


def check(path: str):
    try:
        # kill process which occupies port 8080
        sh.fuser('-k', '8080/tcp')  # get "fuser" command with "apt install psutils" (it is not available on mac)
    except sh.ErrorReturnCode_1:
        pass  # its ok to not find any process to kill
    print(path, end='', flush=True)
    buf = StringIO()
    script = sh.python3(path, _bg=True, _bg_exc=False, _out=buf, _err=buf)
    time.sleep(12)
    output = buf.getvalue()

    try:
        script.terminate()
        script.wait(1)
    except (ProcessLookupError, sh.SignalException_SIGKILL, sh.TimeoutException):
        pass

    if 'Traceback' in output:
        fail(output, 2)
    if 'Error' in output:
        fail(output, 3)

    if 'NiceGUI ready to go' in output:
        print(f' is ok', flush=True)
    else:
        fail(output, 4)


if __name__ == '__main__':
    check('../main.py')
    check('../docs/src/scene_on_click.py')
    check('../docs/src/scene_on_click_with_automation_controls.py')
    check('../docs/src/geofence.py')
    check('../docs/src/path_planning.py')
    check('../docs/src/robot_shape.py')
    check('../docs/src/show_captured_images.py')
    check('../docs/src/remote_operation.py')
    check('../docs/src/logging_config.py')
    check('../docs/src/logging_to_file.py')
    check('../examples/hello_bot/main.py')
    check('../examples/obstacles/main.py')
    check('../rosys/actors/pathplanning/planner_demo.py')

    if has_failures:
        sys.exit(1)
