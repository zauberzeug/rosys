#!/usr/bin/env bash

if [ `uname` == "Linux" ]
then
    export IDF_TOOLS_PATH="$HOME/esp/esp-tools_4.2"
    export IDF_PATH="$HOME/esp/esp-idf_4.2"
    . $IDF_PATH/export.sh
    
    if lshw -C system 2>/dev/null | grep -q "NVIDIA Jetson\|jetson-nano"
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
        sudo echo 0 > /sys/class/gpio/gpio$g0/value
        sudo echo 1 > /sys/class/gpio/gpio$en/value
        sudo echo 0 > /sys/class/gpio/gpio$en/value
        sudo echo $en > /sys/class/gpio/unexport
        sudo echo $g0 > /sys/class/gpio/unexport
    else
        make -j4 flash
    fi
else
    docker run --rm -v $PWD:/project -w /project espressif/idf:v4.2 make -j4 || exit 1
    ./esptool.py \
        --chip esp32 \
        --port /dev/tty.SLAB_USBtoUART \
        --baud 921600 \
        --before default_reset \
        --after hard_reset \
        write_flash \
        -z \
        --flash_mode dio \
        --flash_freq 40m \
        --flash_size detect \
        0x1000 build/bootloader/bootloader.bin \
        0x8000 build/partitions_singleapp.bin \
        0x10000 build/zauberzeug-robot-brain.bin
fi
