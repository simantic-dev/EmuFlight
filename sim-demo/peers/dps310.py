# Infineon DPS310 barometer on I2C (datasheet Rev 1.2) as a ScriptedI2cSlave.
#
# Written from the datasheet: register map and MEAS_CFG ready bits (DS §8),
# coefficient packing (DS §8.11 / Table 18), compensation formula (DS §4.9.1),
# measurement times per oversampling (DS Table 16), start-up times (DS §8.5).
# ArduPilot's AP_Baro_DPS280 driver was read only to learn what it exercises:
# PRODUCT_ID, the 18-byte COEF block, COEF_SRCE bit 7, PRS_CFG/TMP_CFG/CFG_REG/
# MEAS_CFG writes, the polling of PRS_RDY, PSR/TMP reads, RESET 0x09, and the
# undocumented 0x0E/0x0F/0x62 "temperature fix" sequence (accepted, no effect).
#
# Environment input: set `pressure_pa` / `temperature_c`; the raw counts are
# solved from the compensation formula so the firmware recovers those values.
#
# Sensor noise: DS Table 16 "Precision (PaRMS)" per PRC (128x is not listed; the
# 64x value, also the High Precision figure in DS Table 7, is used). Applied as
# Gaussian noise on the raw count, scaled by kP / (dP/dPraw_sc) from DS §4.9.1 so
# the firmware sees the datasheet's Pa RMS. No temperature precision row exists in
# the datasheet, so temperature is noiseless.
import random

NOISE_SEED = 0x0D5310
PRECISION_PA = {1: 2.5, 2: 1.0, 4: 0.5, 8: 0.4, 16: 0.35, 32: 0.3, 64: 0.2, 128: 0.2}

PRODUCT_ID = 0x10               # DS §8.10 (REV_ID 0, PROD_ID 1)
COEF_READY_US = 40_000          # DS §8.5: coefficients readable 40 ms after power-up
SENSOR_READY_US = 12_000        # DS §8.5: sensor initialisation 12 ms
# Measurement time per oversampling rate, ms (DS Table 16).
MEAS_TIME_MS = {1: 3.6, 2: 5.2, 4: 8.4, 8: 14.8, 16: 27.6, 32: 53.2, 64: 104.4, 128: 206.8}
# Compensation scale factors per oversampling (DS Table 9).
KP = {1: 524288, 2: 1572864, 4: 3670016, 8: 7864320, 16: 253952, 32: 516096, 64: 1040384, 128: 2088960}
# A plausible production coefficient set (the values a typical unit reports).
COEF = dict(c0=210, c1=-259, c00=80240, c10=-55328, c01=-2640, c11=1136, c20=-11168, c21=-176, c30=-1104)

R_PSR_B2, R_TMP_B2 = 0x00, 0x03
R_PRS_CFG, R_TMP_CFG, R_MEAS_CFG, R_CFG_REG = 0x06, 0x07, 0x08, 0x09
R_INT_STS, R_FIFO_STS, R_RESET, R_PRODUCT_ID = 0x0A, 0x0B, 0x0C, 0x0D
R_COEF, R_COEF_END, R_COEF_SRCE = 0x10, 0x21, 0x28

MEAS_COEF_RDY, MEAS_SENSOR_RDY, MEAS_TMP_RDY, MEAS_PRS_RDY = 0x80, 0x40, 0x20, 0x10


def _pack(v, bits):
    return v & ((1 << bits) - 1)


def coef_block(c):
    c0, c1 = _pack(c["c0"], 12), _pack(c["c1"], 12)
    c00, c10 = _pack(c["c00"], 20), _pack(c["c10"], 20)
    b = [c0 >> 4, ((c0 & 0xF) << 4) | (c1 >> 8), c1 & 0xFF,
         c00 >> 12, (c00 >> 4) & 0xFF, ((c00 & 0xF) << 4) | (c10 >> 16), (c10 >> 8) & 0xFF, c10 & 0xFF]
    for k in ("c01", "c11", "c20", "c21", "c30"):
        v = _pack(c[k], 16)
        b += [v >> 8, v & 0xFF]
    return bytes(b)


def _s24(v):
    v = int(round(v))
    v = max(-(1 << 23), min((1 << 23) - 1, v))
    return v & 0xFFFFFF


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.pressure_pa = 101325.0
        self.temperature_c = 25.0
        self.t0_us = ctx.now_us
        self.rng = random.Random(NOISE_SEED)
        self.ptr = 0
        self.power_on_reset()
        self.ctx.schedule_oneshot(COEF_READY_US, self.on_timer)

    def power_on_reset(self):
        self.regs = {R_PRS_CFG: 0x00, R_TMP_CFG: 0x00, R_MEAS_CFG: 0x00, R_CFG_REG: 0x00,
                     R_PRODUCT_ID: PRODUCT_ID, R_COEF_SRCE: 0x80}   # temp from MEMS sensor
        self.coef = coef_block(COEF)
        self.raw_p = 0
        self.raw_t = 0
        self.t0_us = self.ctx.now_us

    # ---- physics -> raw counts (inverse of DS §4.9.1) -----------------------
    def _raw(self):
        c = COEF
        kt = KP[1 << (self.regs[R_TMP_CFG] & 0x07)]
        prc = 1 << (self.regs[R_PRS_CFG] & 0x07)
        kp = KP[prc]
        t = (self.temperature_c - c["c0"] / 2.0) / c["c1"]          # Traw_sc
        target = self.pressure_pa

        def f(x):
            return (c["c00"] + x * (c["c10"] + x * (c["c20"] + x * c["c30"]))
                    + t * c["c01"] + t * x * (c["c11"] + x * c["c21"]) - target)
        x = 0.0
        for _ in range(30):                                      # Newton on the cubic
            d = c["c10"] + x * (2 * c["c20"] + 3 * x * c["c30"]) + t * (c["c11"] + 2 * x * c["c21"])
            x -= f(x) / d
        # d is dP/dPraw_sc at the solution, so PRECISION_PA maps to raw counts.
        sigma_raw = PRECISION_PA[prc] * kp / abs(d)
        return _s24(self.rng.gauss(x * kp, sigma_raw)), _s24(t * kt)

    def _meas_ctrl(self):
        return self.regs[R_MEAS_CFG] & 0x07

    def _rate_hz(self):
        return 1 << ((self.regs[R_PRS_CFG] >> 4) & 0x07)             # PM_RATE (DS §8.3)

    def _ready_bits(self):
        el = self.ctx.now_us - self.t0_us
        v = 0
        if el >= SENSOR_READY_US:
            v |= MEAS_SENSOR_RDY
        if el >= COEF_READY_US:
            v |= MEAS_COEF_RDY
        return v

    def on_timer(self):
        ctrl = self._meas_ctrl()
        if ctrl in (5, 6, 7):                                       # continuous modes
            self.raw_p, self.raw_t = self._raw()
            if ctrl in (5, 7):
                self.regs[R_MEAS_CFG] |= MEAS_PRS_RDY
            if ctrl in (6, 7):
                self.regs[R_MEAS_CFG] |= MEAS_TMP_RDY
            self.ctx.schedule_periodic(int(1_000_000 / self._rate_hz()), self.on_timer)
        elif ctrl in (1, 2):                                        # single shot
            self.raw_p, self.raw_t = self._raw()
            self.regs[R_MEAS_CFG] |= MEAS_PRS_RDY if ctrl == 1 else MEAS_TMP_RDY
            self.regs[R_MEAS_CFG] &= ~0x07

    # ---- I2C register-pointer protocol --------------------------------------
    def on_write(self, data):
        if not data:
            return
        self.ptr = data[0]
        for i, v in enumerate(data[1:]):
            self._write(self.ptr + i, v)

    def _write(self, reg, v):
        if reg == R_RESET:
            if (v & 0x0F) == 0x09:                                  # SOFT_RST (DS §8.9)
                self.power_on_reset()
                self.ctx.schedule_oneshot(COEF_READY_US, self.on_timer)
            return
        if reg == R_MEAS_CFG:
            self.regs[R_MEAS_CFG] = (self.regs[R_MEAS_CFG] & 0xF0) | (v & 0x07)
            ctrl = v & 0x07
            if ctrl in (1, 2):
                prc = 1 << (self.regs[R_PRS_CFG if ctrl == 1 else R_TMP_CFG] & 0x07)
                self.ctx.schedule_oneshot(int(MEAS_TIME_MS[prc] * 1000), self.on_timer)
            elif ctrl in (5, 6, 7):
                self.ctx.schedule_periodic(int(1_000_000 / self._rate_hz()), self.on_timer)
            else:
                self.ctx.cancel_timer()
            return
        if reg in (R_PRS_CFG, R_TMP_CFG, R_CFG_REG):
            self.regs[reg] = v
        # 0x0E/0x0F/0x62 and anything else undocumented: accepted, no effect.

    def _read(self, reg):
        if R_PSR_B2 <= reg <= R_PSR_B2 + 2:
            if reg == R_PSR_B2 + 2:
                self.regs[R_MEAS_CFG] &= ~MEAS_PRS_RDY                # cleared once the result is read
            return (self.raw_p >> (8 * (2 - reg))) & 0xFF
        if R_TMP_B2 <= reg <= R_TMP_B2 + 2:
            if reg == R_TMP_B2 + 2:
                self.regs[R_MEAS_CFG] &= ~MEAS_TMP_RDY
            return (self.raw_t >> (8 * (2 - (reg - R_TMP_B2)))) & 0xFF
        if reg == R_MEAS_CFG:
            return self.regs[R_MEAS_CFG] | self._ready_bits()
        if R_COEF <= reg <= R_COEF_END:
            return self.coef[reg - R_COEF] if self._ready_bits() & MEAS_COEF_RDY else 0x00
        return self.regs.get(reg, 0x00)

    def on_read(self, count):
        out = bytes(self._read(self.ptr + i) for i in range(count))
        self.ptr += count
        return out

    def on_finish(self):
        pass
