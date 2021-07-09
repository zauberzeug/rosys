#!/usr/bin/env bash

if [ $# -eq 0 ]
then
    echo "Usage:"
    echo "`basename $0` [build|run|shell|attach|b|r|s|a]"
    exit
fi

name=rosys

args=""
args+=" -v $(pwd):/app"
args+=" -p 80:80"

for cmd in "$@"
do
    case $cmd in
        b | build)
            docker build --tag $name . || exit ;;
        r | run)
            docker run -it $args $name || exit ;;
        s | shell)                     
            docker run -it $args $name /bin/bash || exit ;;
        a | attach)
            docker exec -it $(docker ps -lq) /ros_entrypoint.sh /bin/bash || exit ;;
        *)
            echo "Unknown command \"$cmd\"" ;;
    esac
done
