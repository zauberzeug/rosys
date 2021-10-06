#!/usr/bin/env python3
import asyncio
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
import serial
import os.path


class LineReader:
    # https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522

    def __init__(self, s: serial.Serial):
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
            line = line_reader.readline().decode('utf-8').strip('\n')
            if '^' in line:
                line, check = line.split('^')
                checksum = 0
                for c in line:
                    checksum ^= ord(c)
                if checksum != int(check):
                    print('ERROR: CHECKSUM MISSMATCH ("%s")' % line)
                else:
                    print(line)
            else:
                print(line)
        except UnicodeDecodeError as e:
            print(e)


async def send():
    session = PromptSession()
    while True:
        try:
            with patch_stdout():
                line = await session.prompt_async("> ")
                checksum = 0
                for c in line:
                    checksum ^= ord(c)
                line += '^%d\n' % checksum
                port.write(line.encode('utf-8'))
        except (KeyboardInterrupt, EOFError):
            print("Bye!")
            loop.stop()
            return


def serial_connection() -> serial.Serial:
    for usb_path in [
        "/dev/ttyTHS1",
        "/dev/ttyUSB0",
        "/dev/tty.SLAB_USBtoUART",
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    ]:
        if os.path.exists(usb_path):
            print(f"Connecting to {usb_path}")
            break
    else:
        raise Exception("No serial port found")

    return serial.Serial(usb_path, baudrate=115200, timeout=0.1)


if __name__ == '__main__':
    with serial_connection() as port:
        loop = asyncio.get_event_loop()
        loop.create_task(send())
        loop.run_in_executor(None, receive)
        loop.run_forever()
