#!/usr/bin/env bash

if [ $# -eq 0 ]
then
    echo "Usage:"
    echo "`basename $0` [build|run|log|shell|attach|b|r|l|s|a]"
    exit
fi

name=rosys

esp="/dev/ttyTHS1"

args=""
args+=" -v $(pwd)/scripts:/root/rosys/catkin_ws/src/robot/scripts"
args+=" -v $(pwd)/launch:/root/rosys/catkin_ws/src/robot/launch"
args+=" -v $(pwd)/lizard.txt:/root/rosys/catkin_ws/src/robot/lizard.txt"
args+=" -v $esp:/dev/esp"
args+=" --device=$esp"
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
        l | log | logs)
            docker logs -f --tail 100 $name || exit ;;
        a | attach)
            docker exec -it $(docker ps -lq) /ros_entrypoint.sh /bin/bash || exit ;;
        i | install)
            echo "disabing restart for any containers which may have been configured before"
            docker update --restart=no $(docker ps -a -q) > /dev/null
            echo "configuring running containers to always restart"
            docker update --restart=always $(docker ps -q)
            ;;
        *)
            echo "Unknown command \"$cmd\"" ;;
    esac
done
