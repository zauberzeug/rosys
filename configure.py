#!/usr/bin/env python3
import serial

usb_path = "/dev/tty.SLAB_USBtoUART"

def send(line):

    checksum = 0
    for c in line:
        checksum ^= ord(c)
    port.write(('%s^%d\n' % (line, checksum)).encode())

with serial.Serial(usb_path, baudrate=38400, timeout=1.0) as port:

    send('esp erase')
    with open('config.txt') as f:
        for line in f.read().splitlines():
            if not line.strip().startswith('#'):
                send('+' + line)
    send('esp restart')
