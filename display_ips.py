# ============================================================
#  display_ips.py  —  ST7789 IPS Color Display  (GUI v2)
# ============================================================
#
#  Instrument-panel GUI for PID controller.
#  Adapts to any ST7789 resolution: 128×160, 240×240, 240×320.
#
#  Layout (R0–R5):
#    R0  Temp header  — mode tag | temperature (large) | safety badge
#                       error text | thermometer range bar
#    R1  PWM bar      — gradient fill (green → yellow → red)
#    R2  P/I/D bars   — bidirectional bars centred on zero
#    R3  Gains        — Kp / Ki / Kd one-liner
#    R4  Tuner status — tune count + last action
#    R5  Graph        — scrolling history + 5°C grid + setpoint line
#
#  Pin naming on cheap modules:
#    A0  = DC (Data/Command) — NOT an analogue input
#    SDA = MOSI              — despite the I2C label
#    MOSI / MISO / SD_CS     — SD-card pins, leave unconnected
#
#  invert parameter:
#    True  — most genuine IPS 240×240 panels  (wrong default = white screen)
#    False — non-IPS / 128×160 panels          (try False first)
#
# ============================================================

import struct
import time
import framebuf
from machine import Pin, SPI


# ── RGB565 colour constants ───────────────────────────────────
BLACK    = const(0x0000)
WHITE    = const(0xFFFF)
RED      = const(0xF800)
GREEN    = const(0x07E0)
BLUE     = const(0x001F)
YELLOW   = const(0xFFE0)
ORANGE   = const(0xFD20)
CYAN     = const(0x07FF)
MAGENTA  = const(0xF81F)
GRAY     = const(0x8410)
DARKGRAY = const(0x4208)
DARKBG   = const(0x0861)
SECTBG   = const(0x10A2)
GRAPHBG  = const(0x2104)

_STATE_COLOR = {"OK": GREEN, "WARNING": YELLOW, "CUTOFF": ORANGE, "SHUTDOWN": RED}
_STATE_LABEL = {"OK": " OK ", "WARNING": "WARN", "CUTOFF": "CUT!", "SHUTDOWN": "STOP"}

# ── ST7789 command bytes (module-level) ───────────────────────
# const() defined inside a class body is NOT stored as a real class attribute
# in MicroPython — the compiler optimises it to a local integer.  Subclasses
# therefore cannot access it via self.XXX.  Module level is the correct fix.
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
            (_SLPOUT,    None,                              10),
            (_COLMOD,    b'\x55',                          10),   # RGB565
            (_MADCTL,    b'\x00',                           0),   # normal orientation
            (_PORCTRL,   b'\x0C\x0C\x00\x33\x33',          0),
            (_GCTRL,     b'\x35',                           0),
            (_VCOMS,     b'\x19',                           0),
            (_LCMCTRL,   b'\x2C',                           0),
            (_VDVVRHEN,  b'\x01',                           0),
            (_VRHS,      b'\x12',                           0),
            (_VDVS,      b'\x20',                           0),
            (_FRCTRL2,   b'\x0F',                           0),   # 60 Hz
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
        # framebuf RGB565 is little-endian; ST7789 expects big-endian — swap
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
#  PID Dashboard GUI  —  resolution-adaptive
# ============================================================
class IPSDisplay(ST7789):
    """
    Instrument-panel GUI for PID controller.

    Layout adapts automatically to display size.
    Reference dimensions: 240×240.  Same code runs on 128×160.

    R0  Temp header:  [RUN/CAL tag]  temperature (large)  [safety badge]
                      error text
                      thermometer bar (temp vs setpoint ±15°C range)
    R1  PWM bar:      gradient green→yellow→red + percentage
    R2  P/I/D bars:   bidirectional centre-out bars (P=cyan, I=yellow, D=magenta)
                      output summary line
    R3  Gains:        Kp / Ki / Kd
    R4  Tuner:        tune count + last action
    R5  Graph:        scrolling temperature history, 5°C grid, setpoint line
    """

    STRIPE_H = 64        # override ST7789's 32 — needed for tall R0 header
    GRAPH_HALF_RANGE = 15.0
    _BAR_RANGE = 50.0    # ±range mapped to full bar width for P/I/D

    def __init__(self, sck_pin=18, mosi_pin=23, cs_pin=5,
                 dc_pin=16, rst_pin=17, bl_pin=-1,
                 width=240, height=240,
                 x_offset=0, y_offset=0,
                 spi_freq=40_000_000,
                 spi_id=1,
                 invert=True):
        """
        invert : True  = colour inversion on (required for most IPS 240×240 panels)
                 False = colour inversion off (try first for 128×160 / non-IPS)
        spi_id : 1 for Pico (SPI1), 2 for ESP32 VSPI
        """
        super().__init__(
            sck=sck_pin, mosi=mosi_pin, cs=cs_pin,
            dc=dc_pin,   rst=rst_pin,   bl=bl_pin,
            width=width, height=height,
            spi_freq=spi_freq,
            x_offset=x_offset, y_offset=y_offset,
            spi_id=spi_id, invert=invert,
        )

        self._sc = 2 if width >= 200 else 1
        h, w = height, width

        # ── Region heights proportional to display height (ref = 240 px) ──
        rh0 = max(44, h * 60 // 240)   # R0  temp header
        rh1 = max(14, h * 22 // 240)   # R1  PWM bar
        rh2 = max(36, h * 44 // 240)   # R2  P/I/D bars
        rh3 = max(10, h * 16 // 240)   # R3  gains
        rh4 = max(10, h * 14 // 240)   # R4  tuner
        used = rh0 + rh1 + rh2 + rh3 + rh4
        rh5  = max(0, h - used)        # R5  graph (remainder)

        self._ry = [
            0,
            rh0,
            rh0 + rh1,
            rh0 + rh1 + rh2,
            rh0 + rh1 + rh2 + rh3,
            rh0 + rh1 + rh2 + rh3 + rh4,
        ]
        self._rh = [rh0, rh1, rh2, rh3, rh4, rh5]

        self._badge_w = max(36, w * 80 // 240)   # safety badge width
        self._mode_w  = max(28, w * 52 // 240)   # mode tag width

        self._graph_buf  = bytearray(w)
        self._graph_head = 0
        self._graph_h    = rh5

        print("[IPS] {}x{} GUI v2 | rh={} graph={}px sc={} inv={}".format(
            w, h, self._rh, rh5, self._sc, invert))

    # ─────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────

    def update(self, temperature, setpoint, pid_output,
               p_term=0.0, i_term=0.0, d_term=0.0,
               kp=0.0, ki=0.0, kd=0.0,
               safety_state="OK", tune_count=0,
               last_action="", loop_count=0):
        mode = "CAL" if last_action.startswith("CAL") else "RUN"
        safe = temperature if temperature is not None else setpoint
        self._r0_header(temperature, setpoint, safety_state, mode)
        self._r1_pwm(pid_output)
        self._r2_pid_bars(p_term, i_term, d_term, pid_output)
        self._r3_gains(kp, ki, kd)
        self._r4_tuner(tune_count, last_action)
        if self._graph_h >= 8:
            self._r5_graph(safe, setpoint)

    def menu_screen(self, items, sel):
        """Render settings menu over the full display."""
        sc     = self._sc
        H_HDR  = max(16, 8 * sc + 8)
        H_ITEM = max(12, 8 * sc + 4)
        H_HINT = 12
        fb = self._stripe_fb

        fb.fill_rect(0, 0, self.width, H_HDR, BLUE)
        hdr = "SETTINGS"
        self.draw_text(hdr, (self.width - len(hdr) * 8 * sc) // 2,
                       max(0, (H_HDR - 8 * sc) // 2), WHITE, BLUE, scale=sc)
        self.push_stripe(0, H_HDR)

        for i, item in enumerate(items):
            y0 = H_HDR + i * H_ITEM
            fg = BLACK if i == sel else WHITE
            bg = CYAN  if i == sel else DARKBG
            fb.fill_rect(0, 0, self.width, H_ITEM, bg)
            line = "{} {}".format(item['label'].strip(), item['fmt'])
            self.draw_text(line, 4, max(0, (H_ITEM - 8 * sc) // 2),
                           fg, bg, scale=sc)
            self.push_stripe(y0, H_ITEM)

        y_hint = H_HDR + len(items) * H_ITEM
        fb.fill_rect(0, 0, self.width, H_HINT, SECTBG)
        self.draw_text("SEL=nxt  HOLD=save", 2, 2, GRAY, SECTBG, scale=1)
        self.push_stripe(y_hint, H_HINT)

        y_rest = y_hint + H_HINT
        if y_rest < self.height:
            self.fill_rect(0, y_rest, self.width, self.height - y_rest, BLACK)

    # ─────────────────────────────────────────────────────────
    #  R0  Temperature header
    # ─────────────────────────────────────────────────────────

    def _r0_header(self, temperature, setpoint, safety_state, mode="RUN"):
        H  = self._rh[0]
        W  = self.width
        fb = self._stripe_fb
        fb.fill_rect(0, 0, W, H, DARKBG)

        # Sub-row heights inside R0
        tag_h   = max(10, H * 13 // 60)   # mode + safety tags
        therm_h = max(3,  H *  5 // 60)   # thermometer bar at bottom
        err_h   = max(8,  H *  9 // 60)   # error text
        temp_h  = H - tag_h - err_h - therm_h - 2

        # Temperature text scale: biggest that fits in temp_h
        t_sc = 1
        if temp_h >= 24 and W >= 200:
            t_sc = 3
        elif temp_h >= 16:
            t_sc = 2

        mw = self._mode_w
        bw = self._badge_w

        # ── Mode tag (left) ──
        mc = 0x0340 if mode == "RUN" else 0x8200   # dark teal : dark red
        mt = CYAN   if mode == "RUN" else ORANGE
        fb.fill_rect(0, 0, mw, tag_h, mc)
        fb.rect(0, 0, mw, tag_h, mt)
        ml = "RUN" if mode == "RUN" else "CAL"
        self.draw_text(ml, max(0, (mw - len(ml) * 8) // 2),
                       max(0, (tag_h - 8) // 2), mt, mc, scale=1)

        # ── Safety badge (right) ──
        bc = _STATE_COLOR.get(safety_state, RED)
        bl_lbl = _STATE_LABEL.get(safety_state, "???")
        fb.fill_rect(W - bw, 0, bw, tag_h, bc)
        fb.rect(W - bw, 0, bw, tag_h, BLACK)
        sx = W - bw + (bw - len(bl_lbl) * 8) // 2
        self.draw_text(bl_lbl, max(W - bw, sx),
                       max(0, (tag_h - 8) // 2), BLACK, bc, scale=1)

        # ── Temperature value (centred, large) ──
        err = setpoint - (temperature or setpoint)
        tc  = (GREEN  if abs(err) < 1.0 else
               YELLOW if abs(err) < 5.0 else
               ORANGE if abs(err) < 10.0 else RED)
        if temperature is None:
            tc = GRAY

        t_str  = "---" if temperature is None else "{:.1f}".format(temperature)
        t_full = t_str + "C"
        tw = len(t_full) * 8 * t_sc
        tx = (W - tw) // 2
        ty = tag_h + max(0, (temp_h - 8 * t_sc) // 2)
        self.draw_text(t_full, max(0, tx), max(tag_h, ty), tc, DARKBG, scale=t_sc)

        # ── Error text ──
        err_str = ("Err:{:+.1f}C".format(err)
                   if temperature is not None else "no sensor")
        ey = tag_h + temp_h
        ex = (W - len(err_str) * 8) // 2
        self.draw_text(err_str, max(0, ex), ey, tc, DARKBG, scale=1)

        # ── Thermometer range bar ──
        if temperature is not None:
            bar_y = H - therm_h - 1
            bar_x = 4
            bar_w = W - 8
            g_min = setpoint - self.GRAPH_HALF_RANGE
            g_max = setpoint + self.GRAPH_HALF_RANGE
            g_rng = g_max - g_min

            fb.fill_rect(bar_x, bar_y, bar_w, therm_h, DARKGRAY)

            sp_x   = bar_x + int((setpoint    - g_min) / g_rng * bar_w)
            temp_x = bar_x + int((temperature - g_min) / g_rng * bar_w)
            sp_x   = max(bar_x, min(bar_x + bar_w - 1, sp_x))
            temp_x = max(bar_x, min(bar_x + bar_w - 1, temp_x))

            # Coloured fill between current temp and setpoint
            if temp_x != sp_x:
                fx = min(temp_x, sp_x)
                fw = abs(temp_x - sp_x)
                fc = GREEN if temperature <= setpoint else RED
                fb.fill_rect(fx, bar_y, fw, therm_h, fc)

            fb.vline(sp_x,   bar_y - 1, therm_h + 2, YELLOW)  # setpoint tick
            fb.vline(temp_x, bar_y - 2, therm_h + 4, CYAN)    # current temp tick

        self.push_stripe(0, H)

    # ─────────────────────────────────────────────────────────
    #  R1  PWM duty bar
    # ─────────────────────────────────────────────────────────

    def _r1_pwm(self, duty_pct):
        H  = self._rh[1]
        y0 = self._ry[1]
        W  = self.width
        fb = self._stripe_fb
        fb.fill_rect(0, 0, W, H, SECTBG)
        fb.hline(0, 0, W, DARKGRAY)

        label = "PWM"
        lw    = len(label) * 8 + 4
        self.draw_text(label, 2, max(0, (H - 8) // 2), GRAY, SECTBG, scale=1)

        duty    = max(0.0, min(100.0, duty_pct))
        pct_str = "{:.0f}%".format(duty)
        pw      = len(pct_str) * 8 + 4

        bar_x = lw
        bar_y = 2
        bar_w = W - lw - pw - 4
        bar_h = max(4, H - 4)
        fill  = int(bar_w * duty / 100.0)

        fb.fill_rect(bar_x, bar_y, bar_w, bar_h, DARKGRAY)

        if fill > 0:
            g1 = bar_w * 50 // 100
            g2 = bar_w * 80 // 100
            f1 = min(fill, g1)
            if f1 > 0:
                fb.fill_rect(bar_x, bar_y, f1, bar_h, GREEN)
            if fill > g1:
                f2 = min(fill - g1, g2 - g1)
                fb.fill_rect(bar_x + g1, bar_y, f2, bar_h, YELLOW)
            if fill > g2:
                fb.fill_rect(bar_x + g2, bar_y, fill - g2, bar_h, RED)

        fb.rect(bar_x, bar_y, bar_w, bar_h, GRAY)   # bar border

        self.draw_text(pct_str, W - pw, max(0, (H - 8) // 2), WHITE, SECTBG, scale=1)
        self.push_stripe(y0, H)

    # ─────────────────────────────────────────────────────────
    #  R2  P / I / D bidirectional bars + output summary
    # ─────────────────────────────────────────────────────────

    def _r2_pid_bars(self, p, i, d, output):
        H  = self._rh[2]
        y0 = self._ry[2]
        W  = self.width
        fb = self._stripe_fb
        fb.fill_rect(0, 0, W, H, DARKBG)

        terms     = [("P", p, CYAN), ("I", i, YELLOW), ("D", d, MAGENTA)]
        bar_range = self._BAR_RANGE
        out_h     = 9 if W >= 200 else 8
        bars_h    = H - out_h
        row_h     = bars_h // 3

        lbl_w  = 10
        val_ch = 7 if W >= 200 else 5    # chars for value label
        val_w  = val_ch * 8 + 2
        bar_x  = lbl_w
        bar_w  = W - lbl_w - val_w - 2
        center = bar_x + bar_w // 2
        half_b = bar_w // 2

        for idx, (lbl, val, color) in enumerate(terms):
            ry = idx * row_h
            if idx > 0:
                fb.hline(0, ry, W, SECTBG)

            ly = ry + max(0, (row_h - 8) // 2)
            self.draw_text(lbl, 1, ly, color, DARKBG, scale=1)

            by = ry + 2
            bh = max(3, row_h - 4)
            fb.fill_rect(bar_x, by, bar_w, bh, GRAPHBG)
            fb.vline(center, by, bh, GRAY)     # zero reference line

            # Bidirectional fill from centre
            clamped  = max(-bar_range, min(bar_range, val))
            fill_pix = int(abs(clamped) * half_b // bar_range)
            if fill_pix > 0:
                if val >= 0:
                    fb.fill_rect(center, by, min(fill_pix, half_b), bh, color)
                else:
                    fx = max(bar_x, center - fill_pix)
                    fb.fill_rect(fx, by, center - fx, bh, color)

            vstr = "{:+.1f}".format(val)[:val_ch]
            vx   = W - val_w + max(0, (val_w - len(vstr) * 8) // 2)
            self.draw_text(vstr, max(0, vx), ly, color, DARKBG, scale=1)

        # Output summary line
        out_y  = bars_h
        fb.hline(0, out_y, W, SECTBG)
        oc     = GREEN if output < 50 else (YELLOW if output < 80 else RED)
        os_str = "Out:{:.1f}%".format(output)
        ox     = (W - len(os_str) * 8) // 2
        self.draw_text(os_str, max(0, ox), out_y + 1, oc, DARKBG, scale=1)

        self.push_stripe(y0, H)

    # ─────────────────────────────────────────────────────────
    #  R3  Gains
    # ─────────────────────────────────────────────────────────

    def _r3_gains(self, kp, ki, kd):
        H  = self._rh[3]
        y0 = self._ry[3]
        W  = self.width
        fb = self._stripe_fb
        fb.fill_rect(0, 0, W, H, SECTBG)
        fb.hline(0, 0, W, DARKGRAY)

        if W < 200:
            s = "P{:.1f} I{:.2f} D{:.1f}".format(kp, ki, kd)
        else:
            s = "Kp:{:.3f}  Ki:{:.4f}  Kd:{:.3f}".format(kp, ki, kd)
        self.draw_text(s, 2, max(0, (H - 8) // 2), WHITE, SECTBG, scale=1)
        self.push_stripe(y0, H)

    # ─────────────────────────────────────────────────────────
    #  R4  Tuner status
    # ─────────────────────────────────────────────────────────

    def _r4_tuner(self, tune_count, last_action):
        H  = self._rh[4]
        y0 = self._ry[4]
        W  = self.width
        fb = self._stripe_fb
        fb.fill_rect(0, 0, W, H, SECTBG)
        fb.hline(0, 0, W, DARKGRAY)

        tc     = CYAN if tune_count > 0 else GRAY
        action = last_action[4:].strip() if len(last_action) > 4 else last_action
        s      = "T#{} {}".format(tune_count, action)[:W // 8]
        self.draw_text(s, 2, max(0, (H - 8) // 2), tc, SECTBG, scale=1)
        self.push_stripe(y0, H)

    # ─────────────────────────────────────────────────────────
    #  R5  Scrolling temperature graph
    # ─────────────────────────────────────────────────────────

    def _r5_graph(self, temperature, setpoint):
        gh = self._graph_h
        y0 = self._ry[5]
        W  = self.width
        if gh < 8:
            return

        g_min = setpoint - self.GRAPH_HALF_RANGE
        g_max = setpoint + self.GRAPH_HALF_RANGE
        g_rng = g_max - g_min

        # Update ring buffer
        scaled = int((temperature - g_min) / g_rng * (gh - 1))
        scaled = max(0, min(gh - 1, scaled))
        self._graph_buf[self._graph_head] = scaled
        self._graph_head = (self._graph_head + 1) % W

        # 5°C grid rows
        grid    = []
        t_i_lo  = (int(g_min) // 5) * 5
        t_int   = t_i_lo
        while t_int <= int(g_max) + 5:
            t = float(t_int)
            if g_min <= t <= g_max:
                gp  = gh - 1 - int((t - g_min) / g_rng * (gh - 1))
                gp  = max(0, min(gh - 1, gp))
                grid.append((gp, t, abs(t - setpoint) < 0.5))
            t_int += 5

        # Render in STRIPE_H-high passes
        stripe_h = min(self.STRIPE_H, gh)
        for ss in range(0, gh, stripe_h):
            ah = min(stripe_h, gh - ss)
            se = ss + ah
            fb = self._stripe_fb
            fb.fill_rect(0, 0, W, ah, GRAPHBG)

            # Grid lines
            for gp, t, is_sp in grid:
                if ss <= gp < se:
                    ly = gp - ss
                    if is_sp:
                        fb.hline(0, ly, W, YELLOW)              # solid setpoint line
                    else:
                        for px in range(0, W, 8):
                            fb.hline(px, ly, 4, DARKGRAY)       # dashed 5°C grid
                        lbl = "{:.0f}".format(t)
                        if ly >= 8:
                            self.draw_text(lbl, 2, ly - 8, DARKGRAY, GRAPHBG, scale=1)

            # Temperature trace — cyan line with white hot tip
            prev = None
            for col in range(W):
                idx = (self._graph_head + col) % W
                py  = self._graph_buf[idx]
                ysc = gh - 1 - py

                if prev is not None:
                    lo   = min(prev, ysc);  hi   = max(prev, ysc)
                    s_lo = max(lo,   ss);   s_hi = min(hi,   se - 1)
                    if s_lo <= s_hi:
                        fb.vline(col - 1, s_lo - ss, s_hi - s_lo + 1, CYAN)
                if ss <= ysc < se:
                    fb.pixel(col, ysc - ss, WHITE)   # bright tip
                prev = ysc

            if ss == 0:
                self.draw_text("HISTORY", W - 57, 2, GRAY, GRAPHBG, scale=1)

            self.push_stripe(y0 + ss, ah)
