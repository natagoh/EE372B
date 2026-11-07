# EE372B LDO testbench: sweep the binary-weighted load and read the ADS1220
from machine import Pin, SPI
import sys, time

# Pin map from ee372b-ldo-testbench.kicad_pcb
PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_DRDY = 1, 2, 3, 4, 6
GATE_PINS = (16, 17, 18, 19, 20)            # VG0..VG4 -> Q1..Q5
BRANCH_R  = (250.0, 125.0, 62.5, 31.6, 15.4) # R11, R13, R16||R18, R19, R20

VREF = 2.048  # ADS1220 internal reference


class ADS1220:
    RESET, START, RDATA = 0x06, 0x08, 0x10
    # MUX codes: AINx (+) vs AIN3 (-), AIN3 = LDO ground
    MUX = {0: 0x2, 1: 0x4, 2: 0x5}

    def __init__(self):
        self.spi = SPI(0, baudrate=1_000_000, polarity=0, phase=1,  # SPI mode 1
                       sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
        self.cs = Pin(PIN_CS, Pin.OUT, value=1)
        self.drdy = Pin(PIN_DRDY, Pin.IN, Pin.PULL_UP)

        self._cmd(self.RESET)
        time.sleep_ms(1)
        # reg0: MUX=AIN0-AIN3, gain 1, PGA bypassed
        # reg1: 20 SPS, normal mode, single-shot
        # reg2: internal ref, 50+60 Hz rejection
        # reg3: DRDY only on dedicated pin
        self.regs = bytes([0x21, 0x00, 0x10, 0x00])
        self._xfer(bytes([0x43]) + self.regs)            # WREG addr 0, 4 regs
        readback = self._xfer(b'\x23', 4)                # RREG addr 0, 4 regs
        if readback != self.regs:
            raise OSError("ADS1220 readback mismatch: %s" % readback.hex())

    def _xfer(self, out, nread=0):
        self.cs(0)
        self.spi.write(out)
        data = self.spi.read(nread) if nread else None
        self.cs(1)
        return data

    def _cmd(self, c):
        self._xfer(bytes([c]))

    def read_volts(self, ch, timeout_ms=200):
        self._xfer(bytes([0x40, (self.MUX[ch] << 4) | 0x01]))  # WREG reg0
        self._cmd(self.START)                                  # single-shot conversion
        t0 = time.ticks_ms()
        while self.drdy.value():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise OSError("ADS1220 DRDY timeout")
        b = self._xfer(bytes([self.RDATA]), 3)
        code = (b[0] << 16) | (b[1] << 8) | b[2]
        if code & 0x800000:
            code -= 1 << 24
        return code * VREF / (1 << 23)

    def read_avg(self, ch, n=4):
        return sum(self.read_volts(ch) for _ in range(n)) / n


gates = [Pin(p, Pin.OUT, value=0) for p in GATE_PINS]

def set_load(code):
    for i, g in enumerate(gates):
        g.value((code >> i) & 1)

def load_ohms(code):
    g = sum(1 / BRANCH_R[i] for i in range(5) if (code >> i) & 1)
    return 1 / g if g else float("inf")

def sweep(adc, settle_ms=20, n_avg=4):
    print("code,r_load_ohm,v_out,v_bgr,v_ota,i_load_mA")
    try:
        for code in range(32):
            set_load(code)
            time.sleep_ms(settle_ms)
            v_out = adc.read_avg(0, n_avg)
            v_bgr = adc.read_avg(1, n_avg)
            v_ota = adc.read_avg(2, n_avg)
            r = load_ohms(code)
            i_ma = v_out / r * 1e3 if code else 0.0
            print("%d,%.3f,%.6f,%.6f,%.6f,%.3f" % (code, r, v_out, v_bgr, v_ota, i_ma))
    finally:
        set_load(0)   # never leave the load on
    print("done")


adc = ADS1220()
print("ready: send 's' to sweep, 'r' for a single read")
while True:
    cmd = sys.stdin.readline().strip()
    if cmd == "s":
        sweep(adc)
    elif cmd == "r":
        print([adc.read_volts(ch) for ch in (0, 1, 2)])
