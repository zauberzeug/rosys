#!/usr/bin/env python3
from nicegui import ui
import serial
from line_reader import LineReader

port = serial.Serial("/dev/ttyTHS1", baudrate=115200, timeout=0.1)
line_reader = LineReader(port)

def send(text):

    print(text)

    port.write((text + '\n').encode())
    
text = ui.input()
ui.button('Send', on_click=lambda: send(text.value))

lines = []
output = ui.label().style('white-space:pre;font-family:monospace')

def configure():

    send('esp erase')
    send('+new led led 27')
    send('+set esp.ready=1')
    send('+set esp.24v=1')
    send('+led on')
    send('esp restart')

ui.button('Configure', on_click=configure)

def read():

    global lines

    line = line_reader.readline()
    if line:
        line = line.decode().strip()
        if '^' in line:
            line, check = line.split('^')
            checksum = 0
            for c in line:
                checksum ^= ord(c)
            if checksum != int(check):
                line = line + '^' + check
        line = line.replace('[0;32mI ', '<span style="color:green">')
        line = line.replace('[0m', '</span>')
        lines += [line]
        lines = lines[-20:]
        output.view.inner_html = '\n'.join(lines)

ui.timer(0.01, lambda: read())
