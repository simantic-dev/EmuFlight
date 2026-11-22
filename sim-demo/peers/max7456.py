# AT7456E / MAX7456 analog OSD on SPI (Maxim MAX7456 datasheet 19-3888) as a
# ScriptedSpiSlave.
#
# Written from the datasheet: register map (Table 14), STAT bits (Table 23),
# display memory auto-increment and the 0xFF escape (§ "Display Memory"),
# character memory NVM write/read via CMM 0xA0/0x50 with the 12 ms busy
# window (§ "Character Memory", t_NVM). ArduPilot's AP_OSD_MAX7456 was read
# only to learn its probe: VM0 <- 0x02 (reset), VM0 read back must be 0, font
# check/upload through CMAH/CMAL/CMDO/CMDI, STAT polled for NVR_BUSY and the
# PAL/NTSC/LOS bits, then display writes via DMAH/DMAL/DMM/DMDI.
#
# Video input is a declared board fact: `VIDEO` = "PAL" | "NTSC" | None (loss of sync).
#
# The character NVM is non-volatile: the board boots with whatever the firmware
# last wrote. `NVM_SEED` (next to this file, 256 x 54 bytes, None = blank chip)
# is loaded at start; the Matek seed is ArduPilot's own font table, dumped by
# this peer with MAX7456_NVM_DUMP=<path> set once all 256 characters are written.

import os

VIDEO = "PAL"
NVM_SEED = "max7456_font.nvm"
NVM_WRITE_US = 12_000           # t_NVM character write, max 12 ms
CHAR_BYTES = 54                 # 12x18 pixels x 2 bits = 54 bytes per character
DISPLAY_CHARS = 480             # 16 rows x 30 columns (PAL); NTSC uses the first 390

R_VM0, R_VM1, R_HOS, R_VOS, R_DMM, R_DMAH, R_DMAL, R_DMDI = range(8)
R_CMM, R_CMAH, R_CMAL, R_CMDI = 0x08, 0x09, 0x0A, 0x0B
R_RB0 = 0x10
R_OSDBL = 0x6C
R_STAT = 0xA0                  # read-only addresses carry bit 7 already
R_DMDO = 0xB0
R_CMDO = 0xC0

VM0_RESET = 0x02
DMM_AUTOINC = 0x01
CMM_WRITE_NVM, CMM_READ_NVM = 0xA0, 0x50
STAT_PAL, STAT_NTSC, STAT_LOS, STAT_NVR_BUSY = 0x01, 0x02, 0x04, 0x20


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.nvm = {}                                   # char index -> 54 bytes
        seed = NVM_SEED and os.path.join(os.path.dirname(globals().get("__file__", "")), NVM_SEED)
        if seed and os.path.exists(seed):
            with open(seed, "rb") as f:
                data = f.read()
            self.nvm = {i: data[i * CHAR_BYTES:(i + 1) * CHAR_BYTES] for i in range(256)}
        self.display = bytearray(DISPLAY_CHARS)
        self.shadow = bytearray(CHAR_BYTES)             # character RAM shadow
        self._txn = None
        self.power_on_reset()

    def power_on_reset(self):
        self.regs = {R_OSDBL: 0x10}
        for i in range(16):
            self.regs[R_RB0 + i] = 0x01
        self.dm_addr = 0

    def screen_text(self):
        rows = []
        for r in range(0, DISPLAY_CHARS, 30):
            rows.append("".join(chr(c) if 32 <= c < 127 else "." for c in self.display[r:r + 30]))
        return rows

    # ---- registers ------------------------------------------------------
    def _stat(self):
        v = 0
        if self.ctx.busy:
            v |= STAT_NVR_BUSY
        if VIDEO == "PAL":
            v |= STAT_PAL
        elif VIDEO == "NTSC":
            v |= STAT_NTSC
        else:
            v |= STAT_LOS
        return v

    def read_reg(self, addr):
        if addr == R_STAT:
            return self._stat()
        if addr == R_CMDO:
            return self.shadow[self.regs.get(R_CMAL, 0) % CHAR_BYTES]
        if addr == R_DMDO:
            return self.display[self.dm_addr % DISPLAY_CHARS]
        return self.regs.get(addr & 0x7F, 0)

    def write_reg(self, addr, v):
        if addr == R_VM0 and v & VM0_RESET:
            self.power_on_reset()                       # reset bit self-clears: VM0 reads 0
            return
        if addr == R_DMAH:
            self.regs[R_DMAH] = v
            self.dm_addr = ((v & 0x01) << 8) | (self.dm_addr & 0xFF)
            return
        if addr == R_DMAL:
            self.regs[R_DMAL] = v
            self.dm_addr = (self.dm_addr & 0x100) | v
            return
        if addr == R_DMDI:
            auto = self.regs.get(R_DMM, 0) & DMM_AUTOINC
            if auto and v == 0xFF:                      # escape ends auto-increment mode
                self.regs[R_DMM] &= ~DMM_AUTOINC
                return
            self.display[self.dm_addr % DISPLAY_CHARS] = v
            if auto:
                self.dm_addr = (self.dm_addr + 1) % 512
            return
        if addr == R_CMDI:
            self.shadow[self.regs.get(R_CMAL, 0) % CHAR_BYTES] = v
            return
        if addr == R_CMM:
            chr_ = self.regs.get(R_CMAH, 0)
            if v == CMM_WRITE_NVM:
                self.nvm[chr_] = bytes(self.shadow)
                self.ctx.busy_until(self.ctx.now_us + NVM_WRITE_US)
                if os.environ.get("MAX7456_NVM_DUMP") and len(self.nvm) == 256:
                    with open(os.environ["MAX7456_NVM_DUMP"], "wb") as f:
                        f.write(b"".join(self.nvm[i] for i in range(256)))
            elif v == CMM_READ_NVM:
                self.shadow[:] = self.nvm.get(chr_, bytes(CHAR_BYTES))
            return
        if addr == R_DMM and v & 0x04:                  # clear display memory
            self.display[:] = bytes(DISPLAY_CHARS)
            v &= ~0x04
        self.regs[addr] = v

    # ---- SPI framing: address byte then one data byte -------------------
    def on_select(self):
        self._txn = None

    def on_deselect(self):
        self._txn = None

    def on_finish(self):
        self._txn = None

    def on_transmit(self, b):
        if self._txn is None:
            self._txn = b
            return 0x00
        addr, self._txn = self._txn, None
        if addr & 0x80:
            return self.read_reg(addr) & 0xFF
        self.write_reg(addr, b)
        return 0x00
