#!/usr/bin/env python3
from nicegui import ui
import justpy as jp
import aioserial
from collections import namedtuple
import time

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
inputs = {}


def send(text):
    print(text)
    message_input.value = ''
    port.write((text + '\n').encode())


def configure():
    send('esp erase')
    for s, socket in enumerate(sockets):
        for i, input_ in enumerate(socket.inputs):
            send(f'+new button IN_{s+1}_{i+1} {input_}')
        for o, output in enumerate(socket.outputs):
            send(f'+new led OUT_{s+1}_{o+1} {output}')
        time.sleep(0.1)
    send(f'+new button EN_1 34')
    send(f'+new button EN_2 35')
    send('+set esp.ready=1')
    send('+set esp.24v=1')
    send('+set esp.outputModules=' + ','.join(inputs.keys()))
    send('esp restart')


async def read():
    global lines

    while is_running:
        line = await port.readline_async()
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
            print(line)
            lines += [line]
            lines = lines[-20:]

            if line.startswith('esp get '):
                words = line.split()
                for i, (name, icon) in enumerate(inputs.items()):
                    icon.visible = words[3+i] == '0'

with ui.row():
    with ui.card():
        ui.markdown('#### READY')
        ui.toggle({0: 'off', 1: 'on'}, on_change=lambda e: send(f'set esp.ready={e.value}'))
    with ui.card():
        ui.markdown('#### 24V')
        ui.toggle({0: 'off', 1: 'on'}, on_change=lambda e: send(f'set esp.24v={e.value}'))
    ui.button('Configure', on_click=configure)

with ui.row():
    for s, socket in enumerate(sockets):
        with ui.card():
            ui.markdown(f'#### Socket {s+1}')
            for o, output in enumerate(socket.outputs):
                name = f'OUT_{s+1}_{o+1}'
                with ui.row().style('align-items:center'):
                    ui.label(name)
                    ui.toggle(['on', 'off', 'pulse'], on_change=lambda e, name=name: send(f'{name} {e.value}'))
            for i, input_ in enumerate(socket.inputs):
                name = f'IN_{s+1}_{i+1}'
                with ui.row():
                    ui.label(name)
                    inputs[name] = ui.icon('lens')
                    inputs[name].visible = False
    with ui.card():
        for name in ['EN_1', 'EN_2']:
            with ui.row():
                ui.label(name)
                inputs[name] = ui.icon('lens')
                inputs[name].visible = False

    def reload_inputs():
        send('esp get')
    timer = ui.timer(0.5, reload_inputs)
    with ui.column():
        ui.button('Reload inputs', on_click=reload_inputs)
        ui.checkbox('Auto-reload').bind_value_to(timer, 'active')

message_input = ui.input(placeholder='Send message...', on_change=lambda e: send(e.value))

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
    send('Bye!')  # NOTE: trigger response from ESP to terminate port.readline_async()
