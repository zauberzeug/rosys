#!/usr/bin/env python3
import asyncio
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
import serial
import os.path


class LineReader:
    # https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522

    def __init__(self, s):

        self.buf = bytearray()
        self.s = s

    def readline(self):

        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


def receive():

    line_reader = LineReader(port)

    while True:
        try:
            line = line_reader.readline().decode('utf-8')
            print(line.strip('\n'))
        except Exception as e:
            print(e)


async def send():

    session = PromptSession()

    while True:
        with patch_stdout():
            line = await session.prompt_async("> ")
            port.write(line.encode('utf-8') + b'\n')

for usb_path in [
    "/dev/tty.SLAB_USBtoUART",
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    "/dev/ttyTHS1"]:
    if os.path.exists(usb_path):
        break
else:
    raise Exception("No serial port found")

with serial.Serial(usb_path, baudrate=115200, timeout=0.1) as port:

    loop = asyncio.get_event_loop()
    loop.create_task(send())
    loop.run_in_executor(None, receive)
    loop.run_forever()
