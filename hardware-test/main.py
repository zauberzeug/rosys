#!/usr/bin/env python3
from nicegui import ui
import justpy as jp
import aioserial
import asyncio

port = aioserial.AioSerial('/dev/ttyTHS1', baudrate=115200)

def send(text):

    print(text)
    message_input.value = ''
    port.write((text + '\n').encode())

def configure():

    send('esp erase')
    send('+new led led 27')
    send('+set esp.ready=1')
    send('+set esp.24v=1')
    send('+led on')
    send('esp restart')

async def read():

    global lines

    while True:
        line = await port.readline_async()
        if line:
            line = line.decode().strip()
            print(line)
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
        await asyncio.sleep(0)
    
message_input = ui.input(placeholder="Send message...", on_change=lambda e: send(e.value))

ui.button('Configure', on_click=configure)

with ui.card():
    lines = []
    output = ui.label().style('white-space:pre;font-family:monospace')
    def update_output():
        output.view.inner_html = '\n'.join(lines)
    ui.timer(0.1, update_output)

@jp.app.on_event('startup')
def startup():
    jp.run_task(read())

