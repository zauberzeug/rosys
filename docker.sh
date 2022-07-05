#!/usr/bin/env bash

if [ $# -eq 0 ]
then
    echo "Usage:"
    echo
    echo "  `basename $0` (b | build)   [<containers>]      Build or rebuild"
    echo "  `basename $0` (u | up)      [<containers>]      Create and start"
    echo "  `basename $0` (U | upbuild) [<containers>]      Create and start (force build, pull latest images)"
    echo "  `basename $0` (d | down)    [<containers>]      Stop and remove"
    echo "  `basename $0` (s | start)   [<containers>]      Start"
    echo "  `basename $0` (r | restart) [<containers>]      Restart"
    echo "  `basename $0` (i | install) [<containers>]      Autorestart running containers on failure/reboot"
    echo "  `basename $0` (h | stop)    [<containers>]      Stop (halt)"
    echo "  `basename $0` ps            [<containers>]      List"
    echo "  `basename $0` rm            [<containers>]      Remove"
    echo "  `basename $0` stats                             Show statistics"
    echo
    echo "  `basename $0` (l | log)    <container>            Show log tail (last 100 lines)"
    echo "  `basename $0` (e | exec)   <container> <command>  Execute command"
    echo "  `basename $0` (a | attach) <container>            Attach to container with shell"
    echo
    echo "  `basename $0` prune      Remove all unused containers, networks and images"
    echo "  `basename $0` stopall    Stop all running containers (system-wide!)"
    echo "  `basename $0` killall    Kill all running containers (system-wide!)"
    echo
    echo "Arguments:"
    echo
    echo "  containers    One or more containers (omit to affect all containers)"
    echo "  container     Excactly one container to be affected"
    echo "  command       Command to be executed inside a container"
    exit
fi

compose_args="-p rosys"

os=`uname`
case $os in
    Linux)
        [ -f /etc/nv_tegra_release ] && compose_args="$compose_args -f docker-compose.yml -f docker-compose.jetson.yml"
        ( nvidia-smi > /dev/null 2>&1 ) && compose_args="$compose_args -f docker-compose.yml -f docker-compose.nvidia.yml"
        ;;
    Darwin)
        compose_args="$compose_args -f docker-compose.yml -f docker-compose.mac.yml"
        ;;
    *)
        echo "Unsupported OS: $os"
        exit 1
esac

set -x

cmd=$1
cmd_args=${@:2}
case $cmd in
    b | build)
    	docker-compose $compose_args build $cmd_args
        ;;
    u | up)
        docker-compose $compose_args up -d $cmd_args
        ;;
    U | buildup | upbuild | upb | bup | ub)
        docker-compose $compose_args up -d --build $cmd_args
        ;;
    d | down)
        docker-compose $compose_args down -d $cmd_args
        ;;
    s | start)
        docker-compose $compose_args start $cmd_args
        ;;
    r | restart)
        docker-compose $compose_args restart $cmd_args
        ;;
    i | install)
        echo "disabing restart for any containers which may have been configured before"
        docker update --restart=no $(docker ps -a -q) > /dev/null
        echo "configuring running containers to always restart"
        docker update --restart=always $(docker ps -q)
        ;;
    h | stop)
        docker-compose $compose_args stop $cmd_args
        ;;
    rm)
        docker-compose $compose_args rm $cmd_args
        ;;
    ps)
        docker-compose $compose_args ps $cmd_args
        ;;
    stat | stats)
        docker stats $cmd_args
        ;;
    l | log | logs)
        docker-compose $compose_args logs -f --tail 100 $cmd_args
        ;;
    e | exec)
        docker-compose $compose_args exec $cmd_args
        ;;
    a | attach)
        docker-compose $compose_args exec $cmd_args /bin/bash
        ;;
    prune)
        docker system prune
        ;;
    stopall)
        docker stop $(docker ps -aq)
        ;;
    killall)
        docker kill $(docker ps -aq)
        ;;
    *)
        echo "Unsupported command \"$cmd\""
        exit 1
esac

