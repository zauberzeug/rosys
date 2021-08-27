#!/usr/bin/env bash

if [ $# -ne 1 ]
then
    echo "Usage:"
    echo "`basename $0` user@host"
    exit
fi

host=$1

echo "[1/3] Compiling..."
docker run --rm -v $PWD:/project -w /project espressif/idf:v4.2 make -j4 || exit 1

echo "[2/3] Copying binary files to $host..."
scp build/bootloader/bootloader.bin \
    build/partitions_singleapp.bin \
    build/zauberzeug-robot-brain.bin \
    $host:~/rosys/controller/ || exit 1

echo "[3/3] Flashing ESP..."
ssh -t $host 'cd ~/rosys/controller && ./flash_bin.sh'
