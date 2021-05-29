#!/usr/bin/env bash

# configure pins
en=216
g0=50
sudo echo $en > /sys/class/gpio/export
sudo echo $g0 > /sys/class/gpio/export
sudo echo out > /sys/class/gpio/gpio$en/direction
sudo echo out > /sys/class/gpio/gpio$g0/direction

# bring ESP into flash mode
sudo echo 1 > /sys/class/gpio/gpio$en/value
sudo echo 1 > /sys/class/gpio/gpio$g0/value
sudo echo 0 > /sys/class/gpio/gpio$en/value

# flash ESP with zauberzeug-robot-brain.bin
~/scripts/esptool.py \
	--chip esp32 \
	--port /dev/ttyTHS1 \
	--baud 921600 \
	--before default_reset \
	--after hard_reset \
	write_flash \
	-z \
	--flash_mode dio \
	--flash_freq 40m \
	--flash_size detect \
	0x1000 bootloader.bin \
	0x8000 partitions_singleapp.bin \
	0x10000 zauberzeug-robot-brain.bin

# bring ESP back into normal operation mode
sudo echo 0 > /sys/class/gpio/gpio$g0/value
sudo echo 1 > /sys/class/gpio/gpio$en/value
sudo echo 0 > /sys/class/gpio/gpio$en/value

# release pin configuration
sudo echo $en > /sys/class/gpio/unexport
sudo echo $g0 > /sys/class/gpio/unexport
