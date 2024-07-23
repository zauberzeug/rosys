class BmsMessage:

    def __init__(self, bytes_: list[int]) -> None:
        self.bytes = bytes_
        self.cursor = -1

    @property
    def address(self) -> int:
        return self.bytes[1]

    @property
    def status(self) -> int:
        return self.bytes[2]

    @property
    def length(self) -> int:
        return self.bytes[3]

    @property
    def content(self) -> list[int]:
        return self.bytes[4:-3]

    def __str__(self) -> str:
        address = {0x03: 'STATUS', 0x04: 'VOLTAGE', 0x05: 'VERSION'}[self.address]
        return f'{address}: {" ".join(hex(b) for b in self.content)}'

    def check(self):
        assert self.bytes[0] == 0xdd
        assert (self.bytes[-3] << 8) + self.bytes[-2] == 0xffff - sum(self.content) - self.length - self.status + 1
        assert self.bytes[-1] == 0x77

    def get1(self):
        self.cursor += 1
        return self.content[self.cursor]

    def get2(self):
        self.cursor += 2
        return (self.content[self.cursor - 1] << 8) + self.content[self.cursor]

    def get2_signed(self):
        value = self.get2()
        return value if value < 32768 else value - 65536

    def interpret(self) -> dict:
        d = {}
        if self.address == 0x03:
            d['total voltage'] = self.get2() / 100
            d['current'] = self.get2_signed() / 100
            d['residual capacity'] = self.get2() / 100
            d['nominal capacity'] = self.get2() / 100
            d['cycle life'] = self.get2()
            date = self.get2()
            d['product year'] = 2000 + (date >> 9)
            d['product month'] = (date >> 5) & 0x0F
            d['product day'] = date & 0x1F
            d['balance status'] = self.get2()
            d['balance status high'] = self.get2()
            protection = self.get2()
            protections = [
                'cell block over-voltage',
                'cell block under-voltage',
                'battery over-voltage',
                'battery under-voltage',
                'charging over-temp',
                'charging low-temp',
                'discharging over-temp',
                'discharging low-temp',
                'charging over-current',
                'discharging over-current',
                'short circuit',
                'fore-end IC error',
                'MOS software lock-in',
            ]
            d['protections'] = {p: protection & (1 << i) for i, p in enumerate(protections)}
            version = self.get1()
            d['BMS major version'] = version >> 4
            d['BMS minor version'] = version & 0xf
            d['capacity percent'] = self.get1()
            d['fet'] = ['off', 'charging', 'discharging', 'both'][self.get1()]
            d['number of blocks'] = self.get1()
            num_ntc = self.get1()
            d['temperatures'] = [self.get2() / 10 - 273.15 for _ in range(num_ntc)]
        elif self.address == 0x04:
            d['voltages'] = [self.get2() / 1000 for _ in range(self.length // 2)]
        elif self.address == 0x05:
            d['hardware version'] = ''.join(map(chr, self.content))
        return d
