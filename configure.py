#!/usr/bin/env python3
import serial

usb_path = "/dev/tty.SLAB_USBtoUART"

with serial.Serial(usb_path, baudrate=38400, timeout=1.0) as port:

    send = lambda line: port.write((line + '\n').encode())

    send('esp erase')
    with open('config.txt') as f:
        for line in f.readlines():
            if not line.strip().startswith('#'):
                send('+' + line)
    send('esp restart')
