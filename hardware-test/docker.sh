#!/usr/bin/env bash

if [ $# -eq 0 ]
then
    echo "Usage:"
    echo "`basename $0` [build|run|shell|attach|b|r|s|a]"
    exit
fi

name=hardware-test

args=""
args+=" -v $(pwd):/app"
args+=" -v /dev/ttyTHS1:/dev/ttyTHS1"
args+=" --device=/dev/ttyTHS1"
args+=" -p 9000:80"

for cmd in "$@"
do
    case $cmd in
        b | build)
            docker build --tag $name . || exit ;;
        r | run)
            docker run $args -it $name || exit ;;
        s | shell)
            docker run $args -it $name /bin/bash || exit ;;
        a | attach)
            docker exec -it $(docker ps -lq) /bin/bash || exit ;;
        *)
            echo "Unknown command \"$cmd\"" ;;
    esac
done
