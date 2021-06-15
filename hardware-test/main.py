#!/usr/bin/env python3
from nicegui import ui
import justpy as jp
import aioserial
import asyncio
from collections import namedtuple

port = aioserial.AioSerial('/dev/ttyTHS1', baudrate=115200)

is_running = True

Socket = namedtuple('Socket', ['inputs', 'outputs'])
sockets = [
    Socket(['MCP_A0', '26'], ['MCP_B7', '27']),
    Socket(['36', '13'], ['5', '4']),
    Socket(['MCP_A1', 'MCP_B6'], []),
    Socket(['MCP_A3', 'MCP_A2'], ['MCP_B4', 'MCP_B5']),
    Socket(['MCP_A5', 'MCP_A4'], ['MCP_B2', 'MCP_B3']),
    Socket(['MCP_A7', 'MCP_A6'], ['MCP_B0', 'MCP_B1']),
]

def send(text):

    print(text)
    message_input.value = ''
    port.write((text + '\n').encode())

def configure():

    send('esp erase')
    for s, socket in enumerate(sockets):
        for i, input_ in enumerate(socket.outputs):
            send(f'+new button IN_{s+1}_{i+1} {input_}')
        for o, output in enumerate(socket.outputs):
            send(f'+new led OUT_{s+1}_{o+1} {output}')
    send('+set esp.ready=1')
    send('+set esp.24v=1')
    send('esp restart')

async def read():

    global lines

    while is_running:
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
    
message_input = ui.input(placeholder="Send message...", on_change=lambda e: send(e.value))

ui.button('Configure', on_click=configure)

with ui.row():
    for s, socket in enumerate(sockets):
        with ui.card():
            ui.label(f'Socket {s+1}')
            for o, output in enumerate(socket.outputs):
                name = f'OUT_{s+1}_{o+1}'
                with ui.row():
                    ui.label(name)
                    ui.toggle(['on', 'off', 'pulse'], on_change=lambda e,name=name: send(f'{name} {e.value}'))

with ui.card():
    lines = []
    output = ui.label().style('white-space:pre;font-family:monospace')
    def update_output():
        output.view.inner_html = '\n'.join(lines)
    ui.timer(0.1, update_output)

@jp.app.on_event('startup')
def startup():
    jp.run_task(read())

@jp.app.on_event('shutdown')
def shutdown():
    is_running = False
    send('Bye!') # NOTE: trigger response from ESP to terminate port.readline_async()
