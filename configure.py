#!/usr/bin/env python3
import serial

usb_path = "/dev/ttyTHS1"

with serial.Serial(usb_path, baudrate=38400, timeout=1.0) as port:
    with open('config.txt') as f:
        for line in f.readlines():
            if line.strip().startswith('#'):
                continue
            port.write(line.encode())
