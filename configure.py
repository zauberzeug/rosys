#!/usr/bin/env python3
import serial
import os.path

devs = [
    "/dev/tty.SLAB_USBtoUART",
    "/dev/ttyTHS1",
]
for dev in devs:
    if os.path.exists(dev):
        usb_path = dev
        break
else:
    raise Exception("No device found.")

def send(line):

    checksum = 0
    for c in line:
        checksum ^= ord(c)
    port.write(('%s^%d\n' % (line, checksum)).encode())

with serial.Serial(usb_path, baudrate=38400, timeout=1.0) as port:

    send('esp erase')
    with open('config.txt') as f:
        for line in f.read().splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                send('+' + line)
    send('esp restart')
