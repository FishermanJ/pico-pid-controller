# ============================================================
#  display_oled.py  —  SSD1306 128x64 I2C OLED for Pico
# ============================================================
#
#  Self-contained: SSD1306 driver is included here.
#  No extra libraries needed — just upload this file to Pico.
#
#  Wiring:
#    GP14  →  SDA
#    GP15  →  SCL
#    3.3V  →  VCC
#    GND   →  GND
#
#  Screen layout:
#    Row 0   59.8 C        (large double-size temperature)
#    Row 1   SP:60.0
#    Row 2   Err:+0.20  [OK]
#    Row 3   PWM [=====  ] 75%
#    Row 4   P:+12.5  I:+3.2
#    Row 5   D:-0.8   Kp:5.0
#    Row 6   Ki:0.050  Kd:1.00
#
# ============================================================

from micropython import const
from machine import I2C, Pin
import framebuf

# ── SSD1306 commands ─────────────────────────────────────────
_SET_CONTRAST        = const(0x81)
_SET_ENTIRE_ON       = const(0xa4)
_SET_NORM_INV        = const(0xa6)
_SET_DISP            = const(0xae)
_SET_MEM_ADDR        = const(0x20)
_SET_COL_ADDR        = const(0x21)
_SET_PAGE_ADDR       = const(0x22)
_SET_DISP_START_LINE = const(0x40)
_SET_SEG_REMAP       = const(0xa0)
_SET_MUX_RATIO       = const(0xa8)
_SET_COM_OUT_DIR     = const(0xc0)
_SET_DISP_OFFSET     = const(0xd3)
_SET_COM_PIN_CFG     = const(0xda)
_SET_DISP_CLK_DIV    = const(0xd5)
_SET_PRECHARGE       = const(0xd9)
_SET_VCOM_DESEL      = const(0xdb)
_SET_CHARGE_PUMP     = const(0x8d)


class _SSD1306(framebuf.FrameBuffer):
    """Low-level SSD1306 driver (framebuf-based)."""

    def __init__(self, width, height):
        self.width  = width
        self.height = height
        self.pages  = height // 8
        self.buffer = bytearray(self.pages * width)
        super().__init__(self.buffer, width, height, framebuf.MONO_VLSB)
        self._init()

    def _init(self):
        for cmd in (
            _SET_DISP,
            _SET_MEM_ADDR, 0x00,
            _SET_DISP_START_LINE,
            _SET_SEG_REMAP | 0x01,
            _SET_MUX_RATIO, self.height - 1,
            _SET_COM_OUT_DIR | 0x08,
            _SET_DISP_OFFSET, 0x00,
            _SET_COM_PIN_CFG, 0x02 if (self.width > 2 * self.height) else 0x12,
            _SET_DISP_CLK_DIV, 0x80,
            _SET_PRECHARGE, 0xF1,
            _SET_VCOM_DESEL, 0x30,
            _SET_CONTRAST, 0xFF,
            _SET_ENTIRE_ON,
            _SET_NORM_INV,
            _SET_CHARGE_PUMP, 0x14,
            _SET_DISP | 0x01,
        ):
            self._cmd(cmd)
        self.fill(0)
        self.show()

    def show(self):
        x0, x1 = 0, self.width - 1
        if self.width != 128:
            off = (128 - self.width) // 2
            x0 += off
            x1 += off
        self._cmd(_SET_COL_ADDR);  self._cmd(x0);  self._cmd(x1)
        self._cmd(_SET_PAGE_ADDR); self._cmd(0);   self._cmd(self.pages - 1)
        self._data(self.buffer)

    # Subclasses implement _cmd() and _data()
    def _cmd(self, cmd):  raise NotImplementedError
    def _data(self, buf): raise NotImplementedError


class _SSD1306_I2C(_SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C):
        self._i2c  = i2c
        self._addr = addr
        self._tmp  = bytearray(2)
        self._wl   = [b"\x40", None]
        super().__init__(width, height)

    def _cmd(self, cmd):
        self._tmp[0] = 0x80
        self._tmp[1] = cmd
        self._i2c.writeto(self._addr, self._tmp)

    def _data(self, buf):
        self._wl[1] = buf
        self._i2c.writevto(self._addr, self._wl)


# ── OLEDDisplay — public class used by pico_pid_controller ──

class OLEDDisplay:
    W = 128
    H = 64

    _BADGE = {
        "OK":       " OK ",
        "WARNING":  "WARN",
        "CUTOFF":   "CUT!",
        "SHUTDOWN": "STOP",
    }

    def __init__(self, sda_pin=14, scl_pin=15, i2c_addr=0x3C):
        # GP14/GP15 are on I2C bus 1 on the Pico
        i2c = I2C(1, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400_000)
        self._oled = _SSD1306_I2C(self.W, self.H, i2c, addr=i2c_addr)

        # Small 8-byte buffer for double-size text rendering
        self._cb  = bytearray(8)
        self._cfb = framebuf.FrameBuffer(self._cb, 8, 8, framebuf.MONO_HLSB)

        self._oled.fill(0)
        self._oled.text("PID Controller", 0, 20)
        self._oled.text("  Starting...", 0, 36)
        self._oled.show()
        print("[OLED] Ready  SDA=GP{} SCL=GP{} addr=0x{:02X}".format(
            sda_pin, scl_pin, i2c_addr))

    def update(self, temperature, setpoint, pid_output,
               p_term, i_term, d_term,
               kp, ki, kd,
               safety_state="OK", tune_count=0,
               last_action="", loop_count=0):

        d = self._oled
        d.fill(0)

        # Row 0-1 (y=0..15): large temperature
        t_str = "{:5.1f}".format(temperature) if temperature is not None else " ---.-"
        self._text2(t_str, 0, 0)
        d.text("C", 104, 0)

        # Row 2 (y=16): setpoint
        d.text("SP:{:.1f}".format(setpoint), 0, 16)

        # Row 3 (y=25): error + safety badge
        if temperature is not None:
            d.text("Err:{:+.2f}".format(setpoint - temperature), 0, 25)
        else:
            d.text("Err: N/A", 0, 25)
        badge = self._BADGE.get(safety_state, "????")
        d.text("[{}]".format(badge), 80, 25)

        # Row 4 (y=34): PWM bar
        self._bar(pid_output, 34)

        # Row 5 (y=45): P and I terms
        d.text("P:{:+.1f}".format(p_term), 0, 45)
        d.text("I:{:+.1f}".format(i_term), 64, 45)

        # Row 6 (y=54): D term and gains
        d.text("D:{:+.1f} Kp:{:.1f}".format(d_term, kp), 0, 54)

        d.show()

    def _text2(self, text, x, y):
        """Render text at 2x scale."""
        d = self._oled
        for ch in text:
            for i in range(8):
                self._cb[i] = 0
            self._cfb.text(ch, 0, 0, 1)
            for row in range(8):
                for col in range(8):
                    if self._cfb.pixel(col, row):
                        d.fill_rect(x + col * 2, y + row * 2, 2, 2, 1)
            x += 16

    def _bar(self, duty_pct, y):
        """Draw a PWM duty bar."""
        d = self._oled
        d.text("PWM", 0, y)
        bx, bw, bh = 26, 82, 7
        d.rect(bx, y, bw, bh, 1)
        fill = int((bw - 2) * max(0.0, min(100.0, duty_pct)) / 100.0)
        if fill > 0:
            d.fill_rect(bx + 1, y + 1, fill, bh - 2, 1)
        d.text("{:3.0f}%".format(duty_pct), bx + bw + 2, y)
