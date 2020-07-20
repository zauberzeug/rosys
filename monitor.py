#!/usr/bin/env python3
from cmd import Cmd
import sys
import serial
import os.path
import readline
import threading

histfile = '/tmp/robot_brain_history'
histfile_size = 1000
if os.path.exists(histfile):
    readline.read_history_file(histfile)


class Prompt(Cmd):

    def __init__(self, port):

        Cmd.__init__(self)

        self.prompt = "> "
        self.port = port

    def emptyline(self):

        pass

    def do_EOF(self, arg):

        print()
        sys.exit()

    def default(self, inp):

        self.port.write(('%s\n' % inp).encode('utf-8'))

        readline.set_history_length(histfile_size)
        readline.write_history_file(histfile)

# https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522


class LineReader:

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


with serial.Serial('/dev/tty.SLAB_USBtoUART', baudrate=115200, timeout=0.1) as port:

    def echo():

        line_reader = LineReader(port)

        while True:

            line = line_reader.readline().decode('utf-8')
            print(line.strip())

    thread = threading.Thread(target=echo)
    thread.daemon = True
    thread.start()

    try:
        prompt = Prompt(port)
        prompt.cmdloop()
    except KeyboardInterrupt:
        print(prompt.lastcmd)
        print()
