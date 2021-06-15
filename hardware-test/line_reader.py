import time

# source: https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522

class LineReader:

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
