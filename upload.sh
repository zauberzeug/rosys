#!/usr/bin/env bash

if [ `uname` == "Linux" ]
then
    export IDF_TOOLS_PATH="$HOME/esp/esp-tools_4.0.1"
    export IDF_PATH="$HOME/esp/esp-idf_4.0.1"
    . $IDF_PATH/export.sh
    
    if lshw -C system 2>/dev/null | grep -q "NVIDIA Jetson"
    then
        en=216
        g0=50
        sudo echo $en > /sys/class/gpio/export
        sudo echo $g0 > /sys/class/gpio/export
        sudo echo out > /sys/class/gpio/gpio$en/direction
        sudo echo out > /sys/class/gpio/gpio$g0/direction
        sudo echo 1 > /sys/class/gpio/gpio$en/value
        sudo echo 1 > /sys/class/gpio/gpio$g0/value
        sudo echo 0 > /sys/class/gpio/gpio$en/value
        make -j4 flash
        sudo echo 1 > /sys/class/gpio/gpio$en/value
        sudo echo 0 > /sys/class/gpio/gpio$en/value
        sudo echo $en > /sys/class/gpio/unexport
        sudo echo $g0 > /sys/class/gpio/unexport
    else
        make -j4 flash
    fi
else
    $HOME/.platformio/penv/bin/pio run --target upload
fi
