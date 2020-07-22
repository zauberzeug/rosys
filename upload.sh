#!/usr/bin/env bash

if [ `uname` == "Linux" ]
then
    export IDF_TOOLS_PATH="$HOME/esp/esp-tools_4.0.1"
    export IDF_PATH="$HOME/esp/esp-idf_4.0.1"
    . $IDF_PATH/export.sh
    make -j4 flash
else
    $HOME/.platformio/penv/bin/pio run --target upload
fi
