#!/usr/bin/env python3
import serial
import os.path
import time
import sys

devs = [
    "/dev/tty.SLAB_USBtoUART",
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    "/dev/ttyUSB0",
    "/dev/ttyTHS1",
]
for dev in devs:
    if os.path.exists(dev):
        usb_path = dev
        break
else:
    raise Exception("No device found.")

def send(line):

    print("Sending:", line)

    checksum = 0
    for c in line:
        checksum ^= ord(c)
    port.write(('%s^%d\n' % (line, checksum)).encode())

    time.sleep(0.1)

with serial.Serial(usb_path, baudrate=115200, timeout=1.0) as port:

    send('esp erase')
    with open(sys.argv[1] if len(sys.argv) > 1 else 'config.txt') as f:
        for line in f.read().splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                send('+' + line)
    send('esp restart')
