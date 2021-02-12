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
            if len(data) == 0:
                raise TimeoutError("Could not read from serial port")
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)