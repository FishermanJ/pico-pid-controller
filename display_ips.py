# ============================================================
#  display_ips.py  —  ST7789 IPS Color Display for ESP32 PID
# ============================================================
#
#  Supports:   240 × 240  (square IPS)
#              240 × 320  (tall IPS / ILI9341-style)
#
#  Wiring (SPI):
#    ESP32 3.3V  →  Display VCC / 3.3V
#    ESP32 GND   →  Display GND
#    GPIO 18     →  SCK  (SPI clock)
#    GPIO 23     →  MOSI (SPI data)
#    GPIO  5     →  CS   (chip select)
#    GPIO 16     →  DC   (data / command)
#    GPIO 17     →  RST  (hardware reset)
#    GPIO 15     →  BL   (backlight, optional)
#
#  No external libraries needed — pure MicroPython with
#  machine.SPI and framebuf (both built into ESP32 firmware).
#
# ============================================================
#  SCREEN LAYOUT  (240 × 240 shown — extra 80 px = graph)
#
#  ┌────────────────────────────────────┐  y=  0
#  │  TEMPERATURE   59.8°C   [  OK  ]  │  40 px  (region 0)
#  │  Target: 60.0°C   Err: +0.20°C   │  24 px  (region 1)
#  ├────────────────────────────────────┤  y= 64
#  │  PWM DUTY CYCLE                   │  32 px  (region 2)
#  │  [████████████████░░░░░░] 75%     │
#  ├────────────────────────────────────┤  y= 96
#  │  P: +12.50                        │  16 px  (region 3)
#  │  I:  +3.22                        │  16 px
#  │  D:  -0.80                        │  16 px
#  │  Output: 75.0%                    │  16 px
#  ├────────────────────────────────────┤  y=160
#  │  Kp: 5.000   Ki: 0.050  Kd:1.000 │  20 px  (region 4)
#  │  AutoTuner #3: Offset → Ki↑      │  16 px
#  ├────────────────────────────────────┤  y=196
#  │                                    │  44 px  (region 5 = graph 240×240)
#  │  ──── Temperature History ─────   │  or 124 px if 240×320
#  │  [scrolling line graph]            │
#  └────────────────────────────────────┘  y=240 (or 320)
#
# ============================================================

import struct
import time
import framebuf
from machine import Pin, SPI


# ── RGB565 color constants ────────────────────────────────────
# RGB565: 5 bits red, 6 bits green, 5 bits blue — packed big-endian
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
DARKBG    = const(0x0861)    # Very dark blue, main background
SECTBG    = const(0x10A2)    # Slightly lighter, section backgrounds
GRAPHBG   = const(0x2104)    # Dark gray for graph area

# Safety state → header color mapping
_STATE_COLOR = {
    "OK":       GREEN,
    "WARNING":  YELLOW,
    "CUTOFF":   ORANGE,
    "SHUTDOWN": RED,
}
_STATE_LABEL = {
    "OK":       "  OK  ",
    "WARNING":  " WARN ",
    "CUTOFF":   " CUT! ",
    "SHUTDOWN": " STOP ",
}


# ── ST7789 command bytes (module-level so subclasses can access via self) ──
_SWRESET    = const(0x01)
_SLPOUT     = const(0x11)
_COLMOD     = const(0x3A)
_MADCTL     = const(0x36)
_PORCTRL    = const(0xB2)
_GCTRL      = const(0xB7)
_VCOMS      = const(0xBB)
_LCMCTRL    = const(0xC0)
_VDVVRHEN   = const(0xC2)
_VRHS       = const(0xC3)
_VDVS       = const(0xC4)
_FRCTRL2    = const(0xC6)
_PWRCTRL    = const(0xD0)
_PVGAMCTRL  = const(0xE0)
_NVGAMCTRL  = const(0xE1)
_INVON      = const(0x21)
_INVOFF     = const(0x20)
_NORON      = const(0x13)
_DISPON     = const(0x29)
_CASET      = const(0x2A)
_RASET      = const(0x2B)
_RAMWR      = const(0x2C)


# ============================================================
#  Low-level ST7789 SPI driver
# ============================================================
class ST7789:
    """
    Bare-metal ST7789 / ILI9341 SPI driver.

    Handles hardware init, window addressing, and raw pixel writes.
    Uses a RAM-efficient stripe buffer (240 × STRIPE_H rows) instead
    of a full framebuffer — saves ~100 KB of RAM.
    """

    STRIPE_H = 32   # stripe buffer height in rows — tune down if RAM is tight

    def __init__(self, sck, mosi, cs, dc, rst,
                 bl=-1, width=240, height=240, spi_freq=40_000_000,
                 x_offset=0, y_offset=0, spi_id=1):
        """
        Initialise SPI bus and display hardware.

        Parameters
        ----------
        sck, mosi        : SPI clock and data GPIO pin numbers
        cs               : Chip-Select GPIO number
        dc               : Data/Command GPIO number (high=data, low=command)
        rst              : Hardware reset GPIO number
        bl               : Backlight GPIO number (-1 = no control)
        width, height    : Display resolution in pixels
        spi_freq         : SPI clock in Hz (40 MHz is safe for most displays)
        x_offset, y_offset: Some 240×240 modules need x_offset=0, y_offset=80
        """
        self.width    = width
        self.height   = height
        self._x_off   = x_offset
        self._y_off   = y_offset

        # SPI bus — use spi_id=1 for Pico (SPI0/SPI1), spi_id=2 for ESP32 (VSPI)
        self._spi = SPI(spi_id,
                        baudrate=spi_freq,
                        polarity=0,
                        phase=0,
                        sck=Pin(sck),
                        mosi=Pin(mosi))

        # Control pins
        self._cs  = Pin(cs,  Pin.OUT, value=1)   # CS idle = high
        self._dc  = Pin(dc,  Pin.OUT, value=0)
        self._rst = Pin(rst, Pin.OUT, value=1)
        self._bl  = Pin(bl,  Pin.OUT, value=1) if bl >= 0 else None

        # Stripe framebuffer — 240 × STRIPE_H rows × 2 bytes (RGB565)
        # This is reused for every region to save RAM
        self._stripe_buf = bytearray(width * self.STRIPE_H * 2)
        self._stripe_fb  = framebuf.FrameBuffer(
            self._stripe_buf, width, self.STRIPE_H, framebuf.RGB565
        )

        # 8-byte buffer for scaled text rendering (one character at a time)
        self._char_buf = bytearray(8)
        self._char_fb  = framebuf.FrameBuffer(
            self._char_buf, 8, 8, framebuf.MONO_HLSB
        )

        # Run hardware initialisation sequence
        self._hw_init()

    # ── Hardware init ─────────────────────────────────────────
    def _hw_init(self):
        """
        ST7789 power-on initialisation sequence.

        Order of operations:
          1. Hardware reset via RST pin
          2. Software reset (flush state machine)
          3. Sleep-out (enable oscillator)
          4. Pixel format = RGB565
          5. Timing and gamma registers
          6. Enable inversion (required for IPS modules)
          7. Display on
        """
        # Hardware reset
        self._rst(0); time.sleep_ms(10)
        self._rst(1); time.sleep_ms(120)

        # Command sequence: (cmd, data_bytes_or_None, delay_ms)
        _SEQ = [
            (_SWRESET, None,                           150),
            (_SLPOUT,  None,                            10),
            (_COLMOD,  b'\x55',                         10),  # RGB565
            (_MADCTL,  b'\x00',                          0),  # normal orientation
            (_PORCTRL, b'\x0C\x0C\x00\x33\x33',         0),  # porch
            (_GCTRL,   b'\x35',                          0),  # gate
            (_VCOMS,   b'\x19',                          0),
            (_LCMCTRL, b'\x2C',                          0),
            (_VDVVRHEN,b'\x01',                          0),
            (_VRHS,    b'\x12',                          0),
            (_VDVS,    b'\x20',                          0),
            (_FRCTRL2, b'\x0F',                          0),  # 60 Hz
            (_PWRCTRL, b'\xA4\xA1',                      0),
            (_PVGAMCTRL,
             b'\xD0\x04\x0D\x11\x13\x2B\x3F\x54\x4C\x18\x0D\x0B\x1F\x23', 0),
            (_NVGAMCTRL,
             b'\xD0\x04\x0C\x11\x13\x2C\x3F\x44\x51\x2F\x1F\x1F\x20\x23', 0),
            (_INVON,   None,                             0),  # inversion on — change to _INVOFF if screen is white
            (_NORON,   None,                             0),  # normal mode
            (_DISPON,  None,                            10),  # display on
        ]

        for cmd, data, delay in _SEQ:
            self._cmd(cmd)
            if data:
                self._data(data)
            if delay:
                time.sleep_ms(delay)

        # Blank the screen to avoid showing random pixels on startup
        self.fill_screen(BLACK)

    # ── Low-level SPI primitives ──────────────────────────────
    def _cmd(self, cmd: int) -> None:
        """Send a 1-byte command (DC=low means 'command')."""
        self._dc(0)
        self._cs(0)
        self._spi.write(bytes([cmd]))
        self._cs(1)

    def _data(self, data: bytes) -> None:
        """Send raw data bytes (DC=high means 'data')."""
        self._dc(1)
        self._cs(0)
        self._spi.write(data)
        self._cs(1)

    def _set_window(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """
        Set the rectangular write window on the display.
        All subsequent RAMWR bytes will fill this region left-to-right, top-to-bottom.

        Parameters include the display's x/y hardware offsets (some 240×240
        modules sit at pixel position 0,80 inside a 240×320 controller chip).
        """
        x0 += self._x_off;  x1 += self._x_off
        y0 += self._y_off;  y1 += self._y_off

        self._cmd(_CASET)
        self._data(struct.pack('>HH', x0, x1))   # column start / end

        self._cmd(_RASET)
        self._data(struct.pack('>HH', y0, y1))   # row start / end

        self._cmd(_RAMWR)                         # begin pixel write

    # ── Drawing primitives ────────────────────────────────────
    def fill_screen(self, color: int) -> None:
        """Fill the entire display with one color. Uses a single row buffer."""
        row_buf = struct.pack('>H', color) * self.width
        self._set_window(0, 0, self.width - 1, self.height - 1)
        self._dc(1)
        self._cs(0)
        for _ in range(self.height):
            self._spi.write(row_buf)
        self._cs(1)

    def fill_rect(self, x: int, y: int, w: int, h: int, color: int) -> None:
        """
        Fill a rectangle with one solid color.

        Uses a pre-packed row buffer of width w — efficient because the
        SPI transfer cost is O(w*h), not O(calls).
        """
        if w <= 0 or h <= 0:
            return
        row_buf = struct.pack('>H', color) * w
        self._set_window(x, y, x + w - 1, y + h - 1)
        self._dc(1)
        self._cs(0)
        for _ in range(h):
            self._spi.write(row_buf)
        self._cs(1)

    def push_stripe(self, y_start: int, stripe_h: int) -> None:
        """
        Write the current stripe framebuffer contents to the display
        starting at row y_start, for stripe_h rows.

        The stripe_fb (self._stripe_fb) must have been filled with
        draw calls before this is called.

        Note: framebuf.RGB565 uses little-endian byte order, but ST7789
        wants big-endian. We byte-swap the entire buffer before sending.
        """
        # Byte-swap each 16-bit pixel in-place  (swap [b0,b1] → [b1,b0])
        buf = self._stripe_buf
        n   = self.width * stripe_h * 2
        for i in range(0, n, 2):
            buf[i], buf[i+1] = buf[i+1], buf[i]

        self._set_window(0, y_start, self.width - 1, y_start + stripe_h - 1)
        self._dc(1)
        self._cs(0)
        self._spi.write(memoryview(buf)[:n])
        self._cs(1)

        # Swap back so the framebuf can be re-used for drawing
        for i in range(0, n, 2):
            buf[i], buf[i+1] = buf[i+1], buf[i]

    def draw_text(self, text: str, x: int, y: int,
                  fg: int, bg: int, scale: int = 1) -> None:
        """
        Draw text into the stripe framebuffer using the built-in 8×8 font,
        with optional integer scaling (scale=1 → 8×8, scale=2 → 16×16, etc.).

        How it works:
          1. For each character, render an 8×8 glyph into a tiny 8-byte buffer.
          2. Read each of the 64 source pixels using framebuf.pixel().
          3. If the pixel is ON, draw a scale×scale filled rect in the stripe fb.
          4. If the pixel is OFF, draw a scale×scale filled rect in bg color.
        This approach uses only 8 bytes of extra RAM regardless of string length.

        Parameters
        ----------
        text  : string to draw (clipped if it exceeds stripe width)
        x, y  : top-left pixel position WITHIN THE STRIPE (not the display)
        fg    : foreground RGB565 color
        bg    : background RGB565 color (pass same as bg of the section)
        scale : integer scale factor (1=normal, 2=double, 3=triple)
        """
        fb   = self._stripe_fb
        cb   = self._char_buf
        cfb  = self._char_fb
        max_x = self.width

        for ch in text:
            # Stop drawing if we've gone off the right edge
            if x + 8 * scale > max_x:
                break

            # Clear char buffer and render the character
            for i in range(8):
                cb[i] = 0
            cfb.text(ch, 0, 0, 1)

            # Expand each source pixel into scale×scale dest pixels
            for row in range(8):
                for col in range(8):
                    color = fg if cfb.pixel(col, row) else bg
                    fb.fill_rect(
                        x + col * scale,
                        y + row * scale,
                        scale, scale, color
                    )

            x += 8 * scale


# ============================================================
#  PID Visualization layer (built on top of ST7789)
# ============================================================
class IPSDisplay(ST7789):
    """
    Color PID controller dashboard on ST7789 IPS display.

    Regions (240 × 240 layout):
      R0  y=  0–39   Temperature hero + safety badge
      R1  y= 40–63   Setpoint + error line
      R2  y= 64–95   PWM duty bar
      R3  y= 96–159  P / I / D terms + output
      R4  y=160–195  Gains + auto-tuner last action
      R5  y=196–239  Scrolling temperature history graph
          (extended to y=319 if height=320)
    """

    # Graph Y range: setpoint ± this many °C
    GRAPH_HALF_RANGE = 15.0

    def __init__(self, sck_pin=18, mosi_pin=23, cs_pin=5,
                 dc_pin=16, rst_pin=17, bl_pin=-1,
                 width=240, height=240,
                 x_offset=0, y_offset=0,
                 spi_freq=40_000_000,
                 spi_id=1):
        """
        Create the display driver and run hardware initialisation.

        Parameters match ST7789.__init__ — see that class for full docs.
        spi_id: SPI bus number — 1 for Pico (default), 2 for ESP32 VSPI.
        """
        super().__init__(
            sck=sck_pin, mosi=mosi_pin, cs=cs_pin,
            dc=dc_pin, rst=rst_pin, bl=bl_pin,
            width=width, height=height,
            spi_freq=spi_freq,
            x_offset=x_offset, y_offset=y_offset,
            spi_id=spi_id,
        )

        # Graph ring buffer — one scaled int per horizontal pixel column
        # Stores 0..graph_h values representing temperature history
        self._graph_buf  = bytearray(width)   # 240 bytes
        self._graph_head = 0                   # current write index
        self._graph_h    = height - 196        # pixels available for graph

        print(f"[IPS] {width}×{height} ST7789 ready | graph={self._graph_h}px")

    # ── Public update — called every loop from main() ─────────
    def update(self,
               temperature,           # float °C, or None
               setpoint:   float,
               pid_output: float,
               p_term:     float,
               i_term:     float,
               d_term:     float,
               kp:         float,
               ki:         float,
               kd:         float,
               safety_state: str = "OK",
               tune_count:   int   = 0,
               last_action:  str   = "",
               loop_count:   int   = 0) -> None:
        """
        Redraw all display regions with the latest PID data.
        Each region is drawn independently — only the changed stripes
        are pushed to the display.
        """
        safe_temp = temperature if temperature is not None else setpoint

        self._region_temp_safety(temperature, setpoint, safety_state)
        self._region_error(safe_temp, setpoint)
        self._region_pwm_bar(pid_output)
        self._region_pid_terms(p_term, i_term, d_term, pid_output)
        self._region_gains(kp, ki, kd, tune_count, last_action)
        if self._graph_h >= 16:
            self._region_graph(safe_temp, setpoint)

    def menu_screen(self, items, sel):
        """
        Draw a full-screen settings menu.

        items : list of dicts with 'label' and 'fmt' keys
        sel   : index of the currently highlighted item
        """
        H_HDR  = 24   # header stripe height
        H_ITEM = 20   # per-item stripe height  (scale-2 text = 16 px + 2 px pad each side)
        H_HINT = 20   # hint row at bottom

        fb = self._stripe_fb

        # ── Header ────────────────────────────────────────────
        fb.fill_rect(0, 0, self.width, H_HDR, BLUE)
        self.draw_text("  SETTINGS  ", 0, 4, WHITE, BLUE, scale=2)
        self.push_stripe(0, H_HDR)

        # ── Menu items ────────────────────────────────────────
        for i, item in enumerate(items):
            y0  = H_HDR + i * H_ITEM
            fg  = BLACK if i == sel else WHITE
            bg  = CYAN  if i == sel else DARKBG
            fb.fill_rect(0, 0, self.width, H_ITEM, bg)
            line = "{} {}".format(item['label'].strip(), item['fmt'])
            self.draw_text(line, 4, 2, fg, bg, scale=2)
            self.push_stripe(y0, H_ITEM)

        # ── Hint ──────────────────────────────────────────────
        y_hint = H_HDR + len(items) * H_ITEM
        fb.fill_rect(0, 0, self.width, H_HINT, SECTBG)
        self.draw_text("SEL=next  HOLD=save", 4, 6, GRAY, SECTBG, scale=1)
        self.push_stripe(y_hint, H_HINT)

        # ── Fill remainder with black ──────────────────────────
        y_rest = y_hint + H_HINT
        if y_rest < self.height:
            self.fill_rect(0, y_rest, self.width, self.height - y_rest, BLACK)

    # ── Region renderers ──────────────────────────────────────

    def _region_temp_safety(self, temperature, setpoint, safety_state):
        """
        Region 0 (y=0..39): Large temperature + safety badge.

        Temperature is shown at scale-3 (each char 24×24 px) in CYAN.
        Safety badge fills the right 80 pixels in the safety state color.
        """
        H = 40   # region height
        fb = self._stripe_fb

        # Background fill
        fb.fill_rect(0, 0, self.width, H, DARKBG)

        # Safety badge — right 80 px, full region height
        badge_color = _STATE_COLOR.get(safety_state, RED)
        fb.fill_rect(160, 0, 80, H, badge_color)
        badge_label = _STATE_LABEL.get(safety_state, " ??? ")
        self.draw_text(badge_label, 161, 12, BLACK, badge_color, scale=2)

        # Temperature in scale-3 (24px tall)
        if temperature is None:
            temp_str = "---.-"
        else:
            temp_str = f"{temperature:5.1f}"
        self.draw_text(temp_str, 0, 8, CYAN, DARKBG, scale=2)
        self.draw_text("\xb0C", 84, 16, CYAN, DARKBG, scale=1)

        self.push_stripe(0, H)

    def _region_error(self, temperature, setpoint):
        """Region 1 (y=40..63): Setpoint + error display."""
        H  = 24
        y0 = 40
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)

        err = setpoint - temperature
        self.draw_text(f"Target:{setpoint:6.1f}\xb0C", 4, 4, YELLOW, SECTBG, scale=1)
        err_color = GREEN if abs(err) < 1.0 else (YELLOW if abs(err) < 5.0 else RED)
        self.draw_text(f"Err:{err:+6.2f}\xb0C", 4, 14, err_color, SECTBG, scale=1)

        self.push_stripe(y0, H)

    def _region_pwm_bar(self, duty_pct: float):
        """
        Region 2 (y=64..95): PWM duty cycle bar.

        Bar color shifts green → yellow → red as duty increases.
        0–50% = green, 50–80% = yellow, 80–100% = red.
        """
        H  = 32
        y0 = 64
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)

        self.draw_text("PWM DUTY CYCLE", 4, 2, WHITE, SECTBG, scale=1)

        bar_x = 2
        bar_y = 13
        bar_w = self.width - 4
        bar_h = 14

        # Empty bar background
        fb.fill_rect(bar_x, bar_y, bar_w, bar_h, DARKGRAY)

        # Filled portion
        duty  = max(0.0, min(100.0, duty_pct))
        fill  = int(bar_w * duty / 100.0)
        if fill > 0:
            if duty < 50.0:
                bar_color = GREEN
            elif duty < 80.0:
                bar_color = YELLOW
            else:
                bar_color = RED
            fb.fill_rect(bar_x, bar_y, fill, bar_h, bar_color)

        # Percentage text centered over bar
        pct_str = f"{duty:4.1f}%"
        tx = bar_x + (bar_w - len(pct_str) * 8) // 2
        self.draw_text(pct_str, tx, bar_y + 3, WHITE, bar_color if fill > 0 else DARKGRAY, scale=1)

        self.push_stripe(y0, H)

    def _region_pid_terms(self, p, i, d, output):
        """
        Region 3 (y=96..159): P / I / D individual terms + total output.

        Each term on its own row at scale-2, color-coded:
          P = CYAN,  I = YELLOW,  D = MAGENTA,  Output = WHITE
        """
        H  = 64
        y0 = 96
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, DARKBG)

        # Draw each term as "label: value" at scale-1 with colored value
        rows = [
            (f"P: {p:+8.2f}",      CYAN,    0),
            (f"I: {i:+8.2f}",      YELLOW,  16),
            (f"D: {d:+8.2f}",      MAGENTA, 32),
            (f"Output: {output:5.1f}%", WHITE, 48),
        ]
        for text, color, dy in rows:
            self.draw_text(text, 4, dy, color, DARKBG, scale=1)
            # Draw a thin separator line above each row (except first)
            if dy > 0:
                fb.hline(0, dy - 1, self.width, SECTBG)

        self.push_stripe(y0, H)

    def _region_gains(self, kp, ki, kd, tune_count, last_action):
        """
        Region 4 (y=160..195): PID gains + auto-tuner status line.
        """
        H  = 36
        y0 = 160
        fb = self._stripe_fb
        fb.fill_rect(0, 0, self.width, H, SECTBG)

        # Gains row
        gains = f"Kp:{kp:5.3f}  Ki:{ki:5.3f}  Kd:{kd:5.3f}"
        self.draw_text(gains, 2, 2, WHITE, SECTBG, scale=1)

        # Tuner row — truncate action to fit in 240px / 8px = 30 chars
        tuner_line = f"Tuner#{tune_count}: {last_action}"[:30]
        t_color = CYAN if tune_count > 0 else GRAY
        self.draw_text(tuner_line, 2, 18, t_color, SECTBG, scale=1)

        fb.hline(0, 0, self.width, DARKGRAY)   # top separator

        self.push_stripe(y0, H)

    def _region_graph(self, temperature: float, setpoint: float):
        """
        Region 5 (y=196..end): Scrolling temperature history graph.

        Stores the last `width` temperature readings in a ring buffer.
        Each new call appends the latest temperature and redraws the
        entire graph region by rendering into the stripe buffer first,
        then pushing it to the display in one SPI transfer per stripe.

        Layout:
          - Dark background
          - White dashed setpoint line
          - CYAN line trace (temperature history, newest on right)
        """
        gh  = self._graph_h            # graph height in pixels
        y0  = 196                      # graph starts at y=196 on display

        # Add new temperature reading to ring buffer (scaled to graph pixels)
        g_min = setpoint - self.GRAPH_HALF_RANGE
        g_max = setpoint + self.GRAPH_HALF_RANGE
        g_range = g_max - g_min

        scaled = int((temperature - g_min) / g_range * (gh - 1))
        scaled = max(0, min(gh - 1, scaled))

        self._graph_buf[self._graph_head] = scaled
        self._graph_head = (self._graph_head + 1) % self.width

        # Build setpoint Y pixel position
        sp_y = int((setpoint - g_min) / g_range * (gh - 1))
        sp_y = max(0, min(gh - 1, sp_y))
        sp_row = gh - 1 - sp_y          # flip: 0 = bottom, gh-1 = top

        # --- Render in STRIPE_H-row chunks to avoid allocating a full buffer ---
        stripe_h = min(self.STRIPE_H, gh)

        for stripe_start in range(0, gh, stripe_h):
            actual_h = min(stripe_h, gh - stripe_start)
            fb = self._stripe_fb

            # Clear stripe to graph background
            fb.fill_rect(0, 0, self.width, actual_h, GRAPHBG)

            # Draw setpoint dashed line if it falls inside this stripe
            stripe_end = stripe_start + actual_h
            if stripe_start <= sp_row < stripe_end:
                local_y = sp_row - stripe_start
                for px in range(0, self.width, 4):
                    fb.hline(px, local_y, 2, YELLOW)    # 2px on, 2px off

            # Draw temperature trace for this stripe
            prev_y_screen = None
            for col in range(self.width):
                idx = (self._graph_head + col) % self.width
                px_y = self._graph_buf[idx]              # 0=bottom, gh-1=top
                y_screen = gh - 1 - px_y                 # flip for screen coords

                # Clip to this stripe
                if prev_y_screen is not None:
                    lo = min(prev_y_screen, y_screen)
                    hi = max(prev_y_screen, y_screen)
                    # Draw vertical segment within this stripe
                    s_lo = max(lo, stripe_start)
                    s_hi = min(hi, stripe_end - 1)
                    if s_lo <= s_hi:
                        fb.vline(col - 1, s_lo - stripe_start,
                                 s_hi - s_lo + 1, CYAN)

                if stripe_start <= y_screen < stripe_end:
                    fb.pixel(col, y_screen - stripe_start, CYAN)

                prev_y_screen = y_screen

            # Label "HISTORY" on first stripe
            if stripe_start == 0:
                self.draw_text("TEMP HISTORY", 4, 2, GRAY, GRAPHBG, scale=1)

            self.push_stripe(y0 + stripe_start, actual_h)
