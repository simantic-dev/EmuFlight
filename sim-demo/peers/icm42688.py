# ICM-42688-P 6-axis IMU on SPI (TDK DS-000347) as a ScriptedSpiSlave.
#
# Written from the datasheet: register map and reset values (DS §14), FIFO
# packet 3 layout and the 0x68 header (DS §6.1), ODR/full-scale tables
# (DS §14.35-14.36), 2048-byte FIFO (DS §6.1). ArduPilot's
# AP_InertialSensor_Invensensev3 and INAV's icm42605 driver were read only to
# learn which registers a flight controller exercises (WHO_AM_I, REG_BANK_SEL,
# PWR_MGMT0, *_CONFIG0, FIFO_CONFIG/FIFO_CONFIG1, INTF_CONFIG0, SIGNAL_PATH_RESET,
# FIFO_COUNTH/L + FIFO_DATA bursts, bank-1/2 AAF registers, data registers).
#
# Every timing/scale constant below is a declared, tunable datasheet value.
# Motion input: set `accel_g` / `gyro_dps` / `temperature_c` from a plant or a
# test, in the vehicle body frame ArduPilot uses (x forward, y right, z down).
# The accelerometer measures the reaction to gravity, so the default, at rest
# and level, is -1 g on body z.
from collections import deque
import struct

WHO_AM_I = 0x47                 # DS §14.55 (ICM-42688-P)
FIFO_BYTES = 2048               # DS §6.1
GYRO_STARTUP_US = 45_000        # DS §3.1 gyro start-up time, typ 45 ms
FIFO_RECORD_BYTES = 16          # packet 3: header, accel, gyro, temp, timestamp
TEMP_LSB_PER_C = 2.07           # FIFO 8-bit temperature: T = v/2.07 + 25 (DS §14.2)
TEMP_ZERO_C = 25.0
# ODR field (bits 3:0 of GYRO_CONFIG0 / ACCEL_CONFIG0) -> Hz. DS §14.36.
ODR_HZ = {1: 32000, 2: 16000, 3: 8000, 4: 4000, 5: 2000, 6: 1000, 7: 200, 8: 100,
          9: 50, 10: 25, 11: 12.5, 12: 6.25, 13: 3.125, 14: 1.5625, 15: 500}
# FS_SEL (bits 7:5) -> sensitivity. DS §3.1 / §3.2.
GYRO_LSB_PER_DPS = {0: 16.4, 1: 32.8, 2: 65.5, 3: 131.0, 4: 262.0, 5: 524.3, 6: 1048.6, 7: 2097.2}
ACCEL_LSB_PER_G = {0: 2048, 1: 4096, 2: 8192, 3: 16384}

# Bank 0 registers (DS §14).
R_DEVICE_CONFIG = 0x11
R_INT_CONFIG = 0x14
R_FIFO_CONFIG = 0x16
R_TEMP_DATA1 = 0x1D
R_ACCEL_DATA_X1 = 0x1F
R_GYRO_DATA_X1 = 0x25
R_INT_STATUS = 0x2D
R_FIFO_COUNTH = 0x2E
R_FIFO_COUNTL = 0x2F
R_FIFO_DATA = 0x30
R_SIGNAL_PATH_RESET = 0x4B
R_INTF_CONFIG0 = 0x4C
R_INTF_CONFIG1 = 0x4D
R_PWR_MGMT0 = 0x4E
R_GYRO_CONFIG0 = 0x4F
R_ACCEL_CONFIG0 = 0x50
R_FIFO_CONFIG1 = 0x5F
R_WHO_AM_I = 0x75
R_REG_BANK_SEL = 0x76

RESET_BANK0 = {
    R_INTF_CONFIG0: 0x30,   # FIFO count in bytes, big-endian count and data
    R_INTF_CONFIG1: 0x91,
    R_GYRO_CONFIG0: 0x06,   # +-2000 dps, 1 kHz
    R_ACCEL_CONFIG0: 0x06,  # +-16 g, 1 kHz
    R_WHO_AM_I: WHO_AM_I,
}

INT_STATUS_FIFO_FULL = 0x02
INT_STATUS_DATA_RDY = 0x08


# Body axes as the chip sees them on this board. ArduPilot's hwdef mounts the
# ICM-42688-P at ROTATION_YAW_180, and its Invensensev3 driver takes the chip
# axes as read, so chip = (-x, -y, z) of the body.
BODY_TO_CHIP = (-1, -1, 1)


def _s16(v):
    return max(-32768, min(32767, int(round(v))))


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.accel_g = [0.0, 0.0, -1.0]
        self.gyro_dps = [0.0, 0.0, 0.0]
        self.temperature_c = 25.0
        self._txn = None
        self.power_on_reset()

    # ---- device state --------------------------------------------------
    def power_on_reset(self):
        self.regs = {b: {} for b in range(5)}
        self.regs[0].update(RESET_BANK0)
        self.bank = 0
        self.fifo = deque()
        self.records = 0
        self.int_status = 0
        self.sensors_on_since_us = None
        self.last_sample = None
        self.samples = 0
        self._retime()

    def _r(self, reg, bank=0):
        return self.regs[bank].get(reg, 0)

    def _sensors_on(self):
        pm = self._r(R_PWR_MGMT0)
        # low-noise mode on both (0b11 in bits 1:0 gyro, 3:2 accel)
        return (pm & 0x03) == 0x03 and (pm & 0x0C) == 0x0C

    def _odr_hz(self):
        return ODR_HZ.get(self._r(R_GYRO_CONFIG0) & 0x0F, 1000)

    def _retime(self):
        if self._sensors_on():
            self.ctx.schedule_periodic(int(1_000_000 / self._odr_hz()), self.tick)
        else:
            self.ctx.cancel_timer()

    def _endian(self):
        intf = self._r(R_INTF_CONFIG0)
        data_big = bool(intf & 0x10)     # SENSOR_DATA_ENDIAN
        count_big = bool(intf & 0x20)    # FIFO_COUNT_ENDIAN
        count_rec = bool(intf & 0x40)    # FIFO_COUNT_REC
        return data_big, count_big, count_rec

    def _sample(self):
        g = GYRO_LSB_PER_DPS[(self._r(R_GYRO_CONFIG0) >> 5) & 0x07]
        a = ACCEL_LSB_PER_G[(self._r(R_ACCEL_CONFIG0) >> 5) & 0x03]
        accel = [_s16(s * v * a) for s, v in zip(BODY_TO_CHIP, self.accel_g)]
        gyro = [_s16(s * v * g) for s, v in zip(BODY_TO_CHIP, self.gyro_dps)]
        temp8 = max(-128, min(127, int(round((self.temperature_c - TEMP_ZERO_C) * TEMP_LSB_PER_C))))
        return accel, gyro, temp8

    def tick(self):
        if not self._sensors_on():
            return
        now = self.ctx.now_us
        if self.sensors_on_since_us is None or now - self.sensors_on_since_us < GYRO_STARTUP_US:
            return
        accel, gyro, temp8 = self._sample()
        self.last_sample = (accel, gyro, temp8)
        self.samples += 1
        self.int_status |= INT_STATUS_DATA_RDY
        mode = (self._r(R_FIFO_CONFIG) >> 6) & 0x03   # 0 bypass, 1 stream, 2/3 stop-on-full
        en = self._r(R_FIFO_CONFIG1)
        if mode == 0 or (en & 0x03) == 0:
            return
        big, _, _ = self._endian()
        fmt = ">" if big else "<"
        header = 0x68  # HEADER_ACCEL | HEADER_GYRO | HEADER_TIMESTAMP_ODR (DS §6.1)
        rec = struct.pack(fmt + "B3h3hbH", header, *accel, *gyro, temp8, now & 0xFFFF)
        if len(self.fifo) + len(rec) > FIFO_BYTES:
            if mode == 1:
                for _ in range(FIFO_RECORD_BYTES):
                    self.fifo.popleft()
                self.records -= 1
            else:
                self.int_status |= INT_STATUS_FIFO_FULL
                return
        self.fifo.extend(rec)
        self.records += 1

    # ---- register access ------------------------------------------------
    def read_reg(self, reg):
        if reg == R_REG_BANK_SEL:
            return self.bank
        if self.bank != 0:
            return self._r(reg, self.bank)
        if reg == R_INT_STATUS:
            v, self.int_status = self.int_status, 0   # clear on read
            return v
        if reg in (R_FIFO_COUNTH, R_FIFO_COUNTL):
            _, count_big, count_rec = self._endian()
            n = self.records if count_rec else len(self.fifo)
            hi, lo = (n >> 8) & 0xFF, n & 0xFF
            first_is_high = count_big
            if reg == R_FIFO_COUNTH:
                return hi if first_is_high else lo
            return lo if first_is_high else hi
        if reg == R_FIFO_DATA:
            if not self.fifo:
                return 0xFF
            b = self.fifo.popleft()
            if len(self.fifo) % FIFO_RECORD_BYTES == 0:
                self.records = len(self.fifo) // FIFO_RECORD_BYTES
            return b
        if R_TEMP_DATA1 <= reg <= R_GYRO_DATA_X1 + 5:
            if self.last_sample is None:
                return 0x80 if reg & 1 else 0x00   # -32768: invalid marker
            accel, gyro, temp8 = self.last_sample
            temp16 = int(round((self.temperature_c - TEMP_ZERO_C) * 132.48))  # DS §14.16: 132.48 LSB/C
            words = [temp16, *accel, *gyro]
            big, _, _ = self._endian()
            blob = struct.pack((">" if big else "<") + "7h", *words)
            return blob[reg - R_TEMP_DATA1]
        return self._r(reg)

    def write_reg(self, reg, val):
        if reg == R_REG_BANK_SEL:
            self.bank = val & 0x07
            return
        if self.bank != 0:
            self.regs[self.bank][reg] = val
            return
        if reg == R_DEVICE_CONFIG and val & 0x01:      # SOFT_RESET_CONFIG
            self.power_on_reset()
            return
        if reg == R_SIGNAL_PATH_RESET:
            if val & 0x02:                              # FIFO_FLUSH
                self.fifo.clear()
                self.records = 0
            return
        self.regs[0][reg] = val
        if reg == R_PWR_MGMT0:
            self.sensors_on_since_us = self.ctx.now_us if self._sensors_on() else None
            self._retime()
        elif reg == R_GYRO_CONFIG0:
            self._retime()

    # ---- SPI framing (4-wire: first byte = address, bit 7 = read) ---------
    def on_select(self):
        self._txn = None

    def on_deselect(self):
        self._txn = None

    def on_finish(self):
        self._txn = None

    def on_transmit(self, b):
        if self._txn is None:
            self._txn = [b & 0x7F, bool(b & 0x80)]
            return 0x00
        reg, is_read = self._txn
        if is_read:
            v = self.read_reg(reg)
        else:
            self.write_reg(reg, b)
            v = 0x00
        if reg != R_FIFO_DATA:                          # burst auto-increment, FIFO_DATA excepted
            self._txn[0] = (reg + 1) & 0x7F
        return v & 0xFF
