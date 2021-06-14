#!/usr/bin/env python3
from nicegui import ui
import serial
import time

class LineReader:
    # https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522

    def __init__(self, s, timeout=0.05):

        self.buf = bytearray()
        self.s = s
        self.timeout = timeout

    def readline(self):

        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        start = time.time()
        while time.time() < start + self.timeout and self.s.in_waiting:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

port = serial.Serial("/dev/ttyTHS1", baudrate=115200, timeout=0.1)
line_reader = LineReader(port)

def send(text):

    print(text)

    port.write((text + '\n').encode())
    
text = ui.input()
ui.button('Send', on_click=lambda: send(text.value))

lines = []
output = ui.label().style('white-space:pre')

def configure():

    send('esp erase')
    send('+new led led 27')
    send('+set esp.ready=1')
    send('+set esp.24v=1')
    send('+led on')
    send('esp restart')

ui.button('Configure', on_click=configure)

def read():

    line = line_reader.readline()
    if line:
        global lines
        lines += [line.decode()]
        lines = lines[-20:]
        output.set_text(''.join(lines))

ui.timer(0.01, lambda: read())
