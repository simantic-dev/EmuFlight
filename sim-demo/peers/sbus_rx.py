# S.Bus receiver on the RX2 pad as a ScriptedUartPeer. The F405-Wing has the
# inverter on the board, so this peer emits the frame the UART sees after it:
# plain 100000 baud 8E2 bytes (the firmware programs the UART; the peer only
# supplies bytes and the frame cadence).
#
# Frame (de-facto Futaba S.Bus): 25 bytes every 14 ms -- 0x0F, 22 bytes of
# 16 x 11-bit channels packed LSB first, a flags byte (bit 0 ch17, bit 1 ch18,
# bit 2 frame lost, bit 3 failsafe), 0x00. 172..1811 spans 988..2012 us;
# 992 is the 1500 us centre.

FRAME_US = 14_000
CENTRE, LOW, HIGH = 992, 172, 1811
ARM_AT_FRAME = 500                              # 7 s: after BOOTGRACE and BAD_RX_RECOVERY


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        self.channels = [CENTRE] * 16
        self.channels[2] = LOW                          # throttle down
        self.channels[4] = LOW                          # AUX1 low; EmuFlight refuses ARM_SWITCH-high-at-boot
        self.frame_lost = False
        self.failsafe = False
        self.frames = 0
        # ScriptedUartPeer's ctx: schedule_periodic(us) fires on_timer (no callback arg).
        self.ctx.schedule_periodic(FRAME_US)

    def on_timer(self):
        if self.frames == ARM_AT_FRAME:
            self.channels[4] = HIGH                     # AUX1 high = ARM (aux 0 0 0 1700 2100)
        self.send_frame()

    def frame(self):
        bits = 0
        for i, ch in enumerate(self.channels):
            bits |= (max(0, min(2047, ch)) & 0x7FF) << (11 * i)
        body = bits.to_bytes(22, "little")
        flags = (0x04 if self.frame_lost else 0) | (0x08 if self.failsafe else 0)
        return bytes([0x0F]) + body + bytes([flags, 0x00])

    def send_frame(self):
        self.frames += 1
        self.ctx.send(self.frame())

    def on_byte(self, b):
        pass                                            # nothing flows towards the receiver
