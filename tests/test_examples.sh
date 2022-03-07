#!/usr/bin/env bash

run() {
    timeout 1 sleep 3
    output=`{ timeout --kill-after=8 5 python3 $1; }`
    exitcode=$?
    test $exitcode -eq 124 && exitcode=0 # exitcode 124 is comming from "timeout command above"
    echo $output | grep "JustPy ready to go" > /dev/null || exitcode=1
    echo $output | grep "Traceback" > /dev/null && exitcode=2
    echo $output | grep "Error" > /dev/null && exitcode=3
    if test $exitcode -ne 0; then
        echo "wrong exit code $exitcode. Output was:"
        echo $output
        return 1
    fi
}

check() {
    echo checking $1 ----------
    pushd $(dirname "$1") >/dev/null
    if run $(basename "$1"); then
        echo "ok --------"
        popd > /dev/null
    else
        echo "failed -------"
        popd > /dev/null
        return 1
    fi
}

success=0
pushd ../
check main.py || success=1
check docs/src/scene_on_click.py || success=1
check docs/src/scene_on_click_with_automation_controls.py || success=1
check docs/src/watch_battery_level.py || success=1
check docs/src/path_planning.py || success=1
check docs/src/robot_shape.py || success=1
check docs/src/show_captured_images.py || success=1
check docs/src/remote_operation.py || success=1
check docs/src/logging_config.py || success=1
check docs/src/logging_to_file.py || success=1
check examples/hello_bot/main.py || success=1
check examples/obstacles/main.py || success=1
check rosys/pathplanning/planner_demo.py || success=1
popd
echo exit $success
test $success -eq 0