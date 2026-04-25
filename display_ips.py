# ============================================================
#  display_ips.py  —  ST7789 IPS Color Display
# ============================================================
#
#  Supports any ST7789 resolution: 128×160, 240×240, 240×320.
#  Region heights and text scale are computed from display size
#  at init time, so the same file works on all panel sizes.
#
#  Wiring (SPI, Pico defaults):
#    GP10 → SCK    GP11 → MOSI   GP13 → CS
#    GP8  → DC     GP9  → RST    (BL optional)
#
#  Wiring (SPI, ESP32 defaults):
#    GPIO18 → SCK   GPIO23 → MOSI   GPIO5 → CS
#    GPIO16 → DC    GPIO17 → RST
#
#  Key setting — invert:
#    invert=True   most genuine IPS panels (white background without it)
#    invert=False  non-IPS / some 128×160 panels (black background default)
#    Try False first if the screen stays white after init.
#
# ============================================================

import struct
import time
import framebuf
from machine import Pin, SPI


# ── RGB565 color constants ────────────────────────────────────
BLACK     = const(0x0000)
WHITE     = const(0xFFFF)
RED       = const(0xF800)
GREEN     = const(0x07E0)
BLUE      = const(0x001F)
YELLOW    = const(0xFFE0)
ORANGE    = const(0xFD20)
CYAN      = const(0x07FF)
MAGENTA   = const(0xF81F)
GRAY      = const(0x8410)
DARKGRAY  = const(0x4208)
DARKBG    = const(0x0861)
SECTBG    = const(0x10A2)
GRAPHBG   = const(0x2104)

_STATE_COLOR = {"OK": GREEN, "WARNING": YELLOW, "CUTOFF": ORANGE, "SHUTDOWN": RED}
_STATE_LABEL = {"OK": " OK ", "WARNING": "WARN", "CUTOFF": "CUT!", "SHUTDOWN": "STOP"}

# ── ST7789 command bytes ──────────────────────────────────────
# Must be module-level: const() inside a class is not accessible
# via self in MicroPython (compiler optimises them away).
_SWRESET   = const(0x01)
_SLPOUT    = const(0x11)
_COLMOD    = const(0x3A)
_MADCTL    = const(0x36)
_PORCTRL   = const(0xB2)
_GCTRL     = const(0xB7)
_VCOMS     = const(0xBB)
_LCMCTRL   = const(0xC0)
_VDVVRHEN  = const(0xC2)
_VRHS      = const(0xC3)
_VDVS      = const(0xC4)
_FRCTRL2   = const(0xC6)
_PWRCTRL   = const(0xD0)
_PVGAMCTRL = const(0xE0)
_NVGAMCTRL = const(0xE1)
_INVON     = const(0x21)
_INVOFF    = const(0x20)
_NORON     = const(0x13)
_DISPON    = const(0x29)
_CASET     = const(0x2A)
_RASET     = const(0x2B)
_RAMWR     = const(0x2C)


# ============================================================
#  Low-level ST7789 SPI driver
# ============================================================
class ST7789:
    STRIPE_H = 32

    def __init__(self, sck, mosi, cs, dc, rst,
                 bl=-1, width=240, height=240, spi_freq=40_000_000,
                 x_offset=0, y_offset=0, spi_id=1, invert=True):
        self.width   = width
        self.height  = height
        self._x_off  = x_offset
        self._y_off  = y_offset
        self._invert = invert

        self._spi = SPI(spi_id, baudrate=spi_freq, polarity=0, phase=0,
                        sck=Pin(sck), mosi=Pin(mosi))
        self._cs  = Pin(cs,  Pin.OUT, value=1)
        self._dc  = Pin(dc,  Pin.OUT, value=0)
        self._rst = Pin(rst, Pin.OUT, value=1)
        self._bl  = Pin(bl,  Pin.OUT, value=1) if bl >= 0 else None

        self._stripe_buf = bytearray(width * self.STRIPE_H * 2)
        self._stripe_fb  = framebuf.FrameBuffer(
            self._stripe_buf, width, self.STRIPE_H, framebuf.RGB565)

        self._char_buf = bytearray(8)
        self._char_fb  = framebuf.FrameBuffer(
            self._char_buf, 8, 8, framebuf.MONO_HLSB)

        self._hw_init()

    def _hw_init(self):
        self._rst(0);  time.sleep_ms(10)
        self._rst(1);  time.sleep_ms(120)

        _SEQ = [
            (_SWRESET,   None,                            150),
            (_SLPOUT,    None,                             10),
            (_COLMOD,    b'\x55',                          10),  # RGB565
            (_MADCTL,    b'\x00',                           0),  # normal orientation
            (_PORCTRL,   b'\x0C\x0C\x00\x33\x33',          0),
            (_GCTRL,     b'\x35',                           0),
            (_VCOMS,     b'\x19',                           0),
            (_LCMCTRL,   b'\x2C',                           0),
            (_VDVVRHEN,  b'\x01',                           0),
            (_VRHS,      b'\x12',                           0),
            (_VDVS,      b'\x20',                           0),
            (_FRCTRL2,   b'\x0F',                           0),  # 60 Hz
            (_PWRCTRL,   b'\xA4\xA1',                       0),
            (_PVGAMCTRL,
             b'\xD0\x04\x0D\x11\x13\x2B\x3F\x54\x4C\x18\x0D\x0B\x1F\x23', 0),
            (_NVGAMCTRL,
             b'\xD0\x04\x0C\x11\x13\x2C\x3F\x44\x51\x2F\x1F\x1F\x20\x23', 0),
            (_INVON if self._invert else _INVOFF, None,      0),
            (_NORON,     None,                              0),
            (_DISPON,    None,                             10),
        ]
        for cmd, data, delay in _SEQ:
            self._cmd(cmd)
            if data:
                self._data(data)
            if delay:
                time.sleep_ms(delay)

        self.fill_screen(BLACK)

    def _cmd(self, c):
        self._dc(0);  self._cs(0)
        self._spi.write(bytes([c]))
        self._cs(1)

    def _data(self, d):
        self._dc(1);  self._cs(0)
        self._spi.write(d)
        self._cs(1)

    def _set_window(self, x0, y0, x1, y1):
        x0 += self._x_off;  x1 += self._x_off
        y0 += self._y_off;  y1 += self._y_off
        self._cmd(_CASET);  self._data(struct.pack('>HH', x0, x1))
        self._cmd(_RASET);  self._data(struct.pack('>HH', y0, y1))
        self._cmd(_RAMWR)

    def fill_screen(self, color):
        row = struct.pack('>H', color) * self.width
        self._set_window(0, 0, self.width - 1, self.height - 1)
        self._dc(1);  self._cs(0)
        for _ in range(self.height):
            self._spi.write(row)
        self._cs(1)

    def fill_rect(self, x, y, w, h, color):
        if w <= 0 or h <= 0:
            return
        row = struct.pack('>H', color) * w
        self._set_window(x, y, x + w - 1, y + h - 1)
        self._dc(1);  self._cs(0)
        for _ in range(h):
            self._spi.write(row)
        self._cs(1)

    def push_stripe(self, y_start, stripe_h):
        buf = self._stripe_buf
        n   = self.width * stripe_h * 2
        # framebuf RGB565 is little-endian; ST7789 wants big-endian — swap
        for i in range(0, n, 2):
            buf[i], buf[i+1] = buf[i+1], buf[i]
        self._set_window(0, y_start, self.width - 1, y_start + stripe_h - 1)
        self._dc(1);  self._cs(0)
        self._spi.write(memoryview(buf)[:n])
        self._cs(1)
        for i in range(0, n, 2):
            buf[i], buf[i+1] = buf[i+1], buf[i]

    def draw_text(self, text, x, y, fg, bg, scale=1):
        fb  = self._stripe_fb
        cb  = self._char_buf
        cfb = self._char_fb
        for ch in text:
            if x + 8 * scale > self.width:
                break
            for i in range(8):
                cb[i] = 0
            cfb.text(ch, 0, 0, 1)
            for row in range(8):
                for col in range(8):
                    c = fg if cfb.pixel(col, row) else bg
                    fb.fill_rect(x + col * scale, y + row * scale, scale, scale, c)
            x += 8 * scale


# ============================================================
#  PID Visualization layer — resolution-adaptive
# ============================================================
class IPSDisplay(ST7789):
    """
    Color PID dashboard on ST7789.  Works on 128×160, 240×240, 240×320.

    Region heights are computed proportionally from display height.
    Text scale is 1 for displays narrower than 200 px, 2 for wider.
    """

    GRAPH_HALF_RANGE = 15.0

    def __init__(self, sck_pin=18, mosi_pin=23, cs_pin=5,
                 dc_pin=16, rst_pin=17, bl_pin=-1,
                 width=240, height=240,
                 x_offset=0, y_offset=0,
                 spi_freq=40_000_000,
                 spi_id=1,
                 invert=True):
        """
        invert : True  = enable colour inversion (required for most IPS panels)
                 False = disable inversion (try this if screen stays white)
        spi_id : 1 for Pico (SPI1 on GP10/GP11), 2 for ESP32 VSPI
        """
        super().__init__(
            sck=sck_pin, mosi=mosi_pin, cs=cs_pin,
            dc=dc_pin,   rst=rst_pin,   bl=bl_pin,
            width=width, height=height,
            spi_freq=spi_freq,
            x_offset=x_offset, y_offset=y_offset,
            spi_id=spi_id, invert=invert,
        )

        # Text scale: 2 for wide panels (≥200 px), 1 for narrow (128 px etc.)
        self._sc = 2 if width >= 200 else 1

        # ── Adaptive region layout ────────────────────────────
        # Heights are proportional to display height.
        # Reference values are for 240 px tall display.
        h   = height
        rh0 = h * 40 // 240    # R0  temp + safety badge
        rh1 = h * 24 // 240    # R1  setpoint + error
        rh2 = h * 32 // 240    # R2  PWM duty bar
        rh3 = h * 64 // 240    # R3  P / I / D terms
        rh4 = h * 36 // 240    # R4  gains + tuner status
        used = rh0 + rh1 + rh2 + rh3 + rh4
        rh5 = max(0, h - used) # R5  scrolling temperature graph

        # Y start position of each region
        self._ry = [
            0,
            rh0,
            rh0 + rh1,
            rh0 + rh1 + rh2,
            rh0 + rh1 + rh2 + rh3,
            rh0 + rh1 + rh2 + rh3 + rh4,
        ]
        self._rh = [rh0, rh1, rh2, rh3, rh4, rh5]

        # Badge width proportional to display width (80 px on 240-wide)
        self._badge_w = max(32, width * 80 // 240)

        # Graph ring buffer
        self._graph_buf  = bytearray(width)
        self._graph_head = 0
        self._graph_h    = rh5

        print("[IPS] {}x{} ready | sc={} graph={}px invert={}".format(
            width, height, self._sc, rh5, invert))

    # ── Public API ────────────────────────────────────────────

    def update(self, temperature, setpoint, pid_output,
               p_term, i_term, d_term, kp, ki, kd,
               safety_state="OK", tune_count=0,
               last_action="", loop_count=0):
        safe = temperature if temperature is not None else setpoint
        self._r0_temp_badge(temperature, setpoint, safety_state)
        self._r1_error(safe, setpoint)
        self._r2_pwm(pid_output)
        self._r3_pid(p_term, i_term, d_term, pid_output)
        self._r4_gains(kp, ki, kd, tune_count, last_action)
        if self._graph_h >= 8:
            self._r5_graph(safe, setpoint)

    def menu_screen(self, items, sel):
        sc     = self._sc
        H_HDR  = max(16, 8 * sc + 8)
        H_ITEM = max(12, 8 * sc + 4)
        H_HINT = 14
        fb = self._stripe_fb

        fb.fill_rect(0, 0, self.width, H_HDR, BLUE)
        self.draw_text("SETTINGS", 4, (H_HDR - 8 * sc) // 2, WHITE, BLUE, scale=sc)
        self.push_stripe(0, H_HDR)

        for i, item in enumerate(items):
            y0 = H_HDR + i * H_ITEM
            fg = BLACK if i == sel else WHITE
            bg = CYAN  if i == sel else DARKBG
            fb.fill_rect(0, 0, self.width, H_ITEM, bg)
            line = "{} {}".format(item['label'].strip(), item['fmt'])
            self.draw_text(line, 4, max(0, (H_ITEM - 8 * sc) // 2), fg, bg, scale=sc)
            self.push_stripe(y0, H_ITEM)

        y_hint = H_HDR + len(items) * H_ITEM
        fb.fill_rect(0, 0, self.width, H_HINT, SECTBG)
        self.draw_text("SEL=next  HOLD=save", 2, 3, GRAY, SECTBG, scale=1)
        self.push_stripe(y_hint, H_HINT)

        y_rest = y_hint + H_HINT
        if y_rest < self.height:
            self.fill_rect(0, y_rest, self.width, self.height - y_rest, BLACK)

    # ── Region renderers ──────────────────────────────────────

    def _r0_temp_badge(self, temperature, setpoint, safety_state):
        sc = self._sc
        H  = self._rh[0]
        bw = self._badge_w
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, DARKBG)

        # Safety badge — right bw pixels
        bc = _STATE_COLOR.get(safety_state, RED)
        bl = _STATE_LABEL.get(safety_state, "???")
        fb.fill_rect(self.width - bw, 0, bw, H, bc)
        self.draw_text(bl, self.width - bw + 2, max(0, (H - 8 * sc) // 2),
                       BLACK, bc, scale=sc)

        # Temperature — left side
        t_str = "---.-" if temperature is None else "{:.1f}".format(temperature)
        self.draw_text(t_str, 2, max(0, (H - 8 * sc) // 2), CYAN, DARKBG, scale=sc)

        self.push_stripe(0, H)

    def _r1_error(self, temperature, setpoint):
        H  = self._rh[1]
        y0 = self._ry[1]
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)

        err = setpoint - temperature
        ec  = GREEN if abs(err) < 1.0 else (YELLOW if abs(err) < 5.0 else RED)
        row = H // 2   # half height = y for second row

        if self.width < 200:
            self.draw_text("SP:{:.1f}C".format(setpoint), 2, 0, YELLOW, SECTBG, scale=1)
            self.draw_text("E:{:+.2f}C".format(err),      2, row, ec,     SECTBG, scale=1)
        else:
            self.draw_text("Target:{:6.1f}\xb0C".format(setpoint), 4, 2,  YELLOW, SECTBG, scale=1)
            self.draw_text("Err:{:+6.2f}\xb0C".format(err),        4, 14, ec,     SECTBG, scale=1)

        self.push_stripe(y0, H)

    def _r2_pwm(self, duty_pct):
        H  = self._rh[2]
        y0 = self._ry[2]
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)

        self.draw_text("PWM DUTY", 4, 1, WHITE, SECTBG, scale=1)

        bar_x = 2
        bar_y = 10
        bar_w = self.width - 4
        bar_h = H - bar_y - 2

        duty  = max(0.0, min(100.0, duty_pct))
        fill  = int(bar_w * duty / 100.0)
        fb.fill_rect(bar_x, bar_y, bar_w, max(1, bar_h), DARKGRAY)

        if fill > 0 and bar_h > 0:
            bc = GREEN if duty < 50.0 else (YELLOW if duty < 80.0 else RED)
            fb.fill_rect(bar_x, bar_y, fill, bar_h, bc)
            pct = "{:.0f}%".format(duty)
            tx  = bar_x + (bar_w - len(pct) * 8) // 2
            ty  = bar_y + max(0, (bar_h - 8) // 2)
            self.draw_text(pct, tx, ty, WHITE, bc, scale=1)

        self.push_stripe(y0, H)

    def _r3_pid(self, p, i, d, output):
        H  = self._rh[3]
        y0 = self._ry[3]
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, DARKBG)

        row_h = H // 4
        rows  = [
            ("P:{:+7.2f}".format(p),       CYAN,    0),
            ("I:{:+7.2f}".format(i),       YELLOW,  row_h),
            ("D:{:+7.2f}".format(d),       MAGENTA, row_h * 2),
            ("Out:{:.1f}%".format(output), WHITE,   row_h * 3),
        ]
        for text, color, dy in rows:
            ty = dy + max(0, (row_h - 8) // 2)
            self.draw_text(text, 4, ty, color, DARKBG, scale=1)
            if dy > 0:
                fb.hline(0, dy, self.width, SECTBG)

        self.push_stripe(y0, H)

    def _r4_gains(self, kp, ki, kd, tune_count, last_action):
        H  = self._rh[4]
        y0 = self._ry[4]
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)
        fb.hline(0, 0, self.width, DARKGRAY)

        row_h = max(8, H // 2)
        tc    = CYAN if tune_count > 0 else GRAY

        if self.width < 200:
            # Two short rows for 128 px wide panels (16 chars max per row)
            self.draw_text("Kp:{:.2f} Ki:{:.3f}".format(kp, ki),
                           2, 2, WHITE, SECTBG, scale=1)
            self.draw_text("Kd:{:.2f} T#{}".format(kd, tune_count),
                           2, row_h + 2, tc, SECTBG, scale=1)
        else:
            gains = "Kp:{:.3f} Ki:{:.4f} Kd:{:.3f}".format(kp, ki, kd)
            self.draw_text(gains, 2, 2, WHITE, SECTBG, scale=1)
            tuner = "T#{}: {}".format(tune_count, last_action)[:30]
            self.draw_text(tuner, 2, 18, tc, SECTBG, scale=1)

        self.push_stripe(y0, H)

    def _r5_graph(self, temperature, setpoint):
        gh  = self._graph_h
        y0  = self._ry[5]
        if gh < 8:
            return

        g_min = setpoint - self.GRAPH_HALF_RANGE
        g_max = setpoint + self.GRAPH_HALF_RANGE
        g_rng = g_max - g_min

        scaled = int((temperature - g_min) / g_rng * (gh - 1))
        scaled = max(0, min(gh - 1, scaled))
        self._graph_buf[self._graph_head] = scaled
        self._graph_head = (self._graph_head + 1) % self.width

        sp_y  = int((setpoint - g_min) / g_rng * (gh - 1))
        sp_y  = max(0, min(gh - 1, sp_y))
        sp_row = gh - 1 - sp_y

        stripe_h = min(self.STRIPE_H, gh)
        for ss in range(0, gh, stripe_h):
            ah = min(stripe_h, gh - ss)
            se = ss + ah
            fb = self._stripe_fb
            fb.fill_rect(0, 0, self.width, ah, GRAPHBG)

            if ss <= sp_row < se:
                ly = sp_row - ss
                for px in range(0, self.width, 4):
                    fb.hline(px, ly, 2, YELLOW)

            prev = None
            for col in range(self.width):
                idx     = (self._graph_head + col) % self.width
                px_y    = self._graph_buf[idx]
                y_scr   = gh - 1 - px_y

                if prev is not None:
                    lo = min(prev, y_scr);  hi = max(prev, y_scr)
                    s_lo = max(lo, ss);     s_hi = min(hi, se - 1)
                    if s_lo <= s_hi:
                        fb.vline(col - 1, s_lo - ss, s_hi - s_lo + 1, CYAN)

                if ss <= y_scr < se:
                    fb.pixel(col, y_scr - ss, CYAN)

                prev = y_scr

            if ss == 0:
                self.draw_text("HISTORY", 4, 2, GRAY, GRAPHBG, scale=1)

            self.push_stripe(y0 + ss, ah)
