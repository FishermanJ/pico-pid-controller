# ============================================================
#  Raspberry Pi Pico  —  PID Controller  v4  (Menu UI)
#  MicroPython
# ============================================================
#
#  Three-button OLED menu for changing settings live.
#
#  BUTTONS:
#    GP17  SELECT  — short press: move cursor to next item
#                  — long press  (≥1 s): save all & exit menu
#    GP18  UP      — increase highlighted value  (auto-repeats when held)
#    GP19  DOWN    — decrease highlighted value  (auto-repeats when held)
#
#  MENU ITEMS (shown on OLED when SELECT is pressed):
#    MODE  → RUN / CALIBRATE
#    TEMP  → target temperature  (±0.5 °C per press)
#    Kp    → proportional gain   (±0.1 per press)
#    Ki    → integral gain       (±0.005 per press)
#    Kd    → derivative gain     (±0.1 per press)
#
#  WIRING:
#    GP26 ── NTC (ADC0)     GP16 ── relay/SSR
#    GP25 ── built-in LED   GP17 ── SELECT button ── GND
#    GP14 ── OLED SDA       GP18 ── UP button     ── GND
#    GP15 ── OLED SCL       GP19 ── DOWN button   ── GND
#
#  IPS display (set DISPLAY_TYPE = "IPS" in settings):
#    GP10 ── SCK    GP11 ── MOSI   GP13 ── CS
#    GP8  ── DC     GP9  ── RST    (BL optional)
#    Requires display_ips.py on the Pico filesystem.
#
#  All buttons use internal pull-up — wire directly to GND.
#  OLED driver is self-contained below; IPS requires display_ips.py.
#
# ============================================================


# ============================================================
#  SECTION 1 — SETTINGS
# ============================================================

TARGET_TEMP = 60.0

PIN_SENSOR  = 26
PIN_OUTPUT  = 16
PIN_LED     = 25
PIN_SEL     = 17   # SELECT button
PIN_UP      = 18   # UP button
PIN_DN      = 19   # DOWN button

NTC_R_SERIES = 100_000
NTC_R_AT_25C = 100_000
NTC_B_VALUE  = 3950
ADC_VOLTAGE  = 3.3

RELAY_MODE        = False
PWM_FREQUENCY_HZ  = 1
PWM_DUTY_MIN_PCT  = 0
PWM_DUTY_MAX_PCT  = 100

SAFETY_HARD_MAX_C = 90.0
SAFETY_CUTOFF_PCT = 20.0
SAFETY_WARN_PCT   = 10.0

KP_DEFAULT = 5.0
KI_DEFAULT = 0.05
KD_DEFAULT = 1.0

KP_MIN, KP_MAX = 0.5, 50.0
KI_MIN, KI_MAX = 0.001, 5.0
KD_MIN, KD_MAX = 0.0, 10.0

TEMP_MIN, TEMP_MAX = 20.0, 200.0

RUN_TUNE_EVERY_S = 60
RUN_TUNE_STEP    = 0.03
CAL_TUNE_EVERY_S = 20
CAL_TUNE_STEP    = 0.10
CAL_DURATION_S   = 300

SCORE_DEADBAND = 1.0

STATE_FILE = "pid_state.json"
LOOP_INTERVAL_MS = 500
LOG_EVERY_LOOPS  = 10
SERIAL_PRINT     = True

# ── Display ───────────────────────────────────────────────────
DISPLAY_TYPE = "OLED"       # "OLED" | "IPS" | "NONE"

# OLED settings (SSD1306 I2C — used when DISPLAY_TYPE = "OLED")
OLED_SDA_PIN  = 14
OLED_SCL_PIN  = 15
OLED_I2C_ADDR = 0x3C

# IPS settings (ST7789 SPI — used when DISPLAY_TYPE = "IPS")
# Pico default: SPI1 on GP10/GP11 with CS=GP13, DC=GP8, RST=GP9
IPS_SCK_PIN   = 10
IPS_MOSI_PIN  = 11
IPS_CS_PIN    = 13
IPS_DC_PIN    = 8
IPS_RST_PIN   = 9
IPS_BL_PIN    = -1          # set to backlight pin number if connected, else -1
IPS_WIDTH     = 240
IPS_HEIGHT    = 240
IPS_SPI_ID    = 1           # SPI bus: 1 for Pico, 2 for ESP32


# ============================================================
#  SECTION 2 — IMPORTS
# ============================================================

from micropython import const
from machine import Pin, ADC, I2C
import framebuf, time, math, json, sys, random


def _log(line):
    if SERIAL_PRINT:
        sys.stdout.write(line + "\r\n")


# ============================================================
#  SECTION 3 — SSD1306 OLED DRIVER  (self-contained)
# ============================================================

_CMD = const(0x80)
_DAT = const(0x40)

class _OLED(framebuf.FrameBuffer):
    W = 128;  H = 64

    def __init__(self, i2c, addr=0x3C):
        self._i2c  = i2c
        self._addr = addr
        self._buf  = bytearray(self.W * self.H // 8)
        self._tmp  = bytearray(2)
        self._wl   = [b"\x40", None]
        super().__init__(self._buf, self.W, self.H, framebuf.MONO_VLSB)
        for c in (0xAE,0x20,0x00,0x40,0xA1,0xA8,0x3F,0xC8,
                  0xD3,0x00,0xDA,0x12,0xD5,0x80,0xD9,0xF1,
                  0xDB,0x30,0x81,0xFF,0xA4,0xA6,0x8D,0x14,0xAF):
            self._cmd(c)
        self.fill(0);  self._show()

    def _cmd(self, c):
        self._tmp[0] = _CMD;  self._tmp[1] = c
        self._i2c.writeto(self._addr, self._tmp)

    def _show(self):
        self._cmd(0x21);  self._cmd(0);   self._cmd(127)
        self._cmd(0x22);  self._cmd(0);   self._cmd(7)
        self._wl[1] = self._buf
        self._i2c.writevto(self._addr, self._wl)

    def row(self, y, text, invert=False):
        """Draw one 8-px text row, optionally inverted (white bg, black text)."""
        if invert:
            self.fill_rect(0, y, self.W, 8, 1)
            self.text(text, 0, y, 0)
        else:
            self.text(text, 0, y, 1)

    def show(self):
        self._show()


def make_oled(sda=14, scl=15, addr=0x3C):
    i2c = I2C(1, sda=Pin(sda), scl=Pin(scl), freq=400_000)
    return _OLED(i2c, addr)


# ============================================================
#  SECTION 4 — TEMPERATURE SENSOR
# ============================================================

class TemperatureSensor:
    _MAX = 65535

    def __init__(self, pin):
        self.adc = ADC(Pin(pin))

    def read_celsius(self):
        try:
            raw = self.adc.read_u16()
            if raw <= 10 or raw >= self._MAX - 10:
                return None
            v    = (raw / self._MAX) * ADC_VOLTAGE
            r    = NTC_R_SERIES * (ADC_VOLTAGE - v) / v
            tk   = 1.0 / (1.0/298.15 + math.log(r/NTC_R_AT_25C)/NTC_B_VALUE)
            return tk - 273.15
        except:
            return None


# ============================================================
#  SECTION 5 — PID CONTROLLER
# ============================================================

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp;  self.ki = ki;  self.kd = kd
        self.setpoint = setpoint
        self._integ = 0.0;  self._prev = 0.0;  self._t = None
        self.last_p = self.last_i = self.last_d = 0.0

    def compute(self, t):
        now = time.ticks_ms()
        dt  = LOOP_INTERVAL_MS/1000. if self._t is None else \
              max(0.001, time.ticks_diff(now, self._t)/1000.)
        self._t = now
        err = self.setpoint - t
        p   = self.kp * err
        self._integ += err * dt
        if self.ki > 1e-4:
            self._integ = max(-100./self.ki, min(100./self.ki, self._integ))
        i = self.ki * self._integ
        d = self.kd * (err - self._prev) / dt
        self._prev = err
        self.last_p = p;  self.last_i = i;  self.last_d = d
        return max(0., min(100., p + i + d))

    def reset(self):
        self._integ = 0.;  self._prev = 0.;  self._t = None

    def load(self, kp, ki, kd):
        self.kp = kp;  self.ki = ki;  self.kd = kd;  self.reset()


# ============================================================
#  SECTION 6 — OUTPUT DRIVER  (soft-PWM)
# ============================================================

class OutputDriver:
    def __init__(self, pin, relay, freq, dmin, dmax):
        self.relay = relay;  self.dmin = dmin;  self.dmax = dmax
        self.duty  = 0.;     self.pin  = Pin(pin, Pin.OUT)
        self.pin.value(0)
        if not relay:
            self.period = int(1000/freq);  self._cs = time.ticks_ms()

    def set(self, pct):
        d = 0. if pct <= 0.5 else max(self.dmin, min(self.dmax, pct))
        self.duty = d
        if self.relay:
            self.pin.value(1 if d > 50. else 0)

    def tick(self):
        if self.relay:  return
        el = time.ticks_diff(time.ticks_ms(), self._cs)
        if el >= self.period:
            self._cs = time.ticks_ms();  el = 0
        on = int(self.duty/100.*self.period)
        self.pin.value(1 if on > 0 and el < on else 0)

    def off(self):
        self.duty = 0.;  self.pin.value(0)


# ============================================================
#  SECTION 7 — SAFETY SYSTEM
# ============================================================

class SafetySystem:
    OK="OK"; WARNING="WARNING"; CUTOFF="CUTOFF"; SHUTDOWN="SHUTDOWN"

    def __init__(self, sp):
        self.warn_c   = sp * (1 + SAFETY_WARN_PCT/100.)
        self.cutoff_c = sp * (1 + SAFETY_CUTOFF_PCT/100.)
        self.hard_max = SAFETY_HARD_MAX_C
        self.state    = self.OK;  self.blocked = False

    def check(self, t):
        if t is None:
            self.state = self.WARNING;  self.blocked = True;  return False
        if t >= self.hard_max:
            if self.state != self.SHUTDOWN: print("[SAFETY] SHUTDOWN {:.1f}C".format(t))
            self.state = self.SHUTDOWN;  self.blocked = True;  return False
        if t >= self.cutoff_c:
            if self.state not in (self.CUTOFF, self.SHUTDOWN):
                print("[SAFETY] CUTOFF {:.1f}C".format(t))
            self.state = self.CUTOFF;  self.blocked = True;  return False
        if t >= self.warn_c:
            self.state = self.WARNING;  self.blocked = False;  return True
        if self.state == self.CUTOFF:
            print("[SAFETY] Resumed {:.1f}C".format(t))
        self.state = self.OK;  self.blocked = False;  return True


# ============================================================
#  SECTION 8 — STATE MANAGER
# ============================================================

class StateManager:
    MODE_RUN = "RUN";  MODE_CAL = "CALIBRATE"

    def __init__(self):
        self.best_kp = KP_DEFAULT;  self.best_ki = KI_DEFAULT
        self.best_kd = KD_DEFAULT;  self.best_score = float('inf')
        self.mode     = self.MODE_RUN;  self.cal_count = 0
        self._load()

    def propose(self, kp, ki, kd, score):
        if score < self.best_score:
            self.best_kp = kp;  self.best_ki = ki;  self.best_kd = kd
            self.best_score = score;  self._save()
            print("[STATE] New best score={:.3f}".format(score))
            return True
        return False

    def set_mode(self, m):
        self.mode = m;  self._save()

    def factory_reset(self):
        self.best_kp = KP_DEFAULT;  self.best_ki = KI_DEFAULT
        self.best_kd = KD_DEFAULT;  self.best_score = float('inf')
        self.mode = self.MODE_CAL;  self._save()
        print("[STATE] Factory reset")

    def _save(self):
        try:
            with open(STATE_FILE,'w') as f:
                json.dump({'kp':self.best_kp,'ki':self.best_ki,'kd':self.best_kd,
                           'score':self.best_score,'mode':self.mode,
                           'cal':self.cal_count}, f)
        except Exception as e:
            print("[STATE] Save err:", e)

    def _load(self):
        try:
            with open(STATE_FILE,'r') as f:  d = json.load(f)
            self.best_kp    = d.get('kp', KP_DEFAULT)
            self.best_ki    = d.get('ki', KI_DEFAULT)
            self.best_kd    = d.get('kd', KD_DEFAULT)
            self.best_score = d.get('score', float('inf'))
            self.mode       = d.get('mode', self.MODE_RUN)
            self.cal_count  = d.get('cal', 0)
            if self.best_score < float('inf'):
                print("[STATE] Loaded score={:.3f}".format(self.best_score))
        except:
            print("[STATE] No state file — using defaults")


# ============================================================
#  SECTION 9 — PERFORMANCE TRACKER + AUTO-TUNER
# ============================================================

class PerfTracker:
    def __init__(self, window_s):
        self.window_s = window_s
        self._errs = [];  self._signs = 0;  self._ps = 0
        self._t0   = time.ticks_ms()

    def record(self, error):
        self._errs.append(max(0., abs(error) - SCORE_DEADBAND))
        s = 1 if error > 0 else (-1 if error < 0 else 0)
        if s != 0 and self._ps != 0 and s != self._ps:
            self._signs += 1
        self._ps = s

    def ready(self):
        return time.ticks_diff(time.ticks_ms(), self._t0)/1000. >= self.window_s \
               and len(self._errs) >= 5

    def score(self):
        if not self._errs:  return float('inf')
        mae = sum(self._errs) / len(self._errs)
        osc = self._signs / max(self.window_s/60., 0.01)
        return mae + 0.5 * osc

    def reset(self):
        self._errs = [];  self._signs = 0;  self._t0 = time.ticks_ms()


class AutoTuner:
    def __init__(self, pid, every_s, step):
        self.pid = pid;  self.every = every_s;  self.step = step
        self._errs = [];  self._sc = 0;  self._ps = 0
        self._t    = time.ticks_ms()
        self.count = 0;  self.last = "none"

    def record(self, temperature):
        e = self.pid.setpoint - temperature
        self._errs.append(e)
        s = 1 if e > 0 else (-1 if e < 0 else 0)
        if s != 0 and self._ps != 0 and s != self._ps:  self._sc += 1
        self._ps = s
        if time.ticks_diff(time.ticks_ms(), self._t)/1000. >= self.every:
            self._tune()

    def _tune(self):
        if len(self._errs) < 5:  return
        n   = len(self._errs)
        avg = sum(self._errs)/n
        mae = sum(abs(e) for e in self._errs)/n
        osc = self._sc / max(self.every/60., 0.01)
        p   = self.pid
        if osc > 4:
            p.kp = max(KP_MIN, min(KP_MAX, p.kp*(1-self.step)))
            p.kd = max(KD_MIN, min(KD_MAX, p.kd*(1+self.step)))
            self.last = "osc Kp- Kd+"
        elif abs(avg) > 1.:
            p.ki = max(KI_MIN, min(KI_MAX, p.ki*(1+self.step)))
            self.last = "offset Ki+"
        elif mae > 5. and osc < 1:
            p.kp = max(KP_MIN, min(KP_MAX, p.kp*(1+self.step*.5)))
            self.last = "slow Kp+"
        else:
            self.last = "ok"
        self.count += 1
        print("[TUNE#{}] {}  Kp={:.3f} Ki={:.4f} Kd={:.3f}".format(
            self.count, self.last, p.kp, p.ki, p.kd))
        self._errs = [];  self._sc = 0;  self._t = time.ticks_ms()

    def reconfigure(self, every_s, step):
        self.every = every_s;  self.step = step
        self._errs = [];  self._t = time.ticks_ms()


# ============================================================
#  SECTION 10 — THREE-BUTTON INPUT
# ============================================================

class Buttons:
    """
    SELECT (GP17): short press → 'sel'   long press → 'sel_long'
    UP     (GP18): press/hold  → 'up'    (auto-repeats every 300 ms when held)
    DOWN   (GP19): press/hold  → 'dn'    (auto-repeats every 300 ms when held)
    """
    DEBOUNCE_MS = 50
    LONG_MS     = 1000
    REPEAT_MS   = 300   # auto-repeat interval for UP / DOWN

    def __init__(self, pin_sel, pin_up, pin_dn):
        self._pins  = [Pin(pin_sel, Pin.IN, Pin.PULL_UP),
                       Pin(pin_up,  Pin.IN, Pin.PULL_UP),
                       Pin(pin_dn,  Pin.IN, Pin.PULL_UP)]
        self._raw   = [1, 1, 1]      # last debounced state (1=open)
        self._down  = [False]*3      # currently held?
        self._down_t = [0]*3         # when was it pressed
        self._rep_t  = [0]*3         # last repeat fire
        self._chg_t  = [0]*3         # last change time (debounce)

    def read(self):
        events = []
        now    = time.ticks_ms()

        for i, pin in enumerate(self._pins):
            raw = pin.value()   # 0 = pressed (pulled to GND)

            # ── Debounce ─────────────────────────────────────
            if raw != self._raw[i]:
                if time.ticks_diff(now, self._chg_t[i]) < self.DEBOUNCE_MS:
                    continue   # noise, ignore
                self._raw[i]  = raw
                self._chg_t[i] = now

                if raw == 0:    # pressed
                    self._down[i]  = True
                    self._down_t[i] = now
                    self._rep_t[i]  = now
                    # UP / DOWN fire immediately on press
                    if i == 1:  events.append('up')
                    if i == 2:  events.append('dn')

                else:           # released
                    if self._down[i]:
                        self._down[i] = False
                        if i == 0:   # SELECT fires on release
                            held = time.ticks_diff(now, self._down_t[i])
                            events.append('sel_long' if held >= self.LONG_MS else 'sel')

            # ── Auto-repeat for UP / DOWN while held ─────────
            elif raw == 0 and self._down[i] and i > 0:
                if time.ticks_diff(now, self._rep_t[i]) >= self.REPEAT_MS:
                    events.append('up' if i == 1 else 'dn')
                    self._rep_t[i] = now

        return events


# ============================================================
#  SECTION 11 — DISPLAY + MENU CONTROLLER
# ============================================================

class Display:
    """
    Unified display wrapper — routes to OLED (SSD1306) or IPS (ST7789)
    depending on DISPLAY_TYPE.

    OLED menu layout (128×64, 8-px font):
      y= 0  "--- SETTINGS ---"
      y= 8  item 0  (inverted when selected)
      ...
      y=56  hint row

    IPS menu: full-color 240×240 screen rendered via IPSDisplay.menu_screen().
    """

    _BADGE = {"OK":" OK ","WARNING":"WARN","CUTOFF":"CUT!","SHUTDOWN":"STOP"}

    def __init__(self):
        self._oled = None
        self._ips  = None

        if DISPLAY_TYPE == "OLED":
            try:
                self._oled = make_oled(OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_ADDR)
                print("[DISPLAY] OLED ready")
            except Exception as e:
                print("[DISPLAY] OLED failed:", e)

        elif DISPLAY_TYPE == "IPS":
            try:
                from display_ips import IPSDisplay
                self._ips = IPSDisplay(
                    sck_pin  = IPS_SCK_PIN,
                    mosi_pin = IPS_MOSI_PIN,
                    cs_pin   = IPS_CS_PIN,
                    dc_pin   = IPS_DC_PIN,
                    rst_pin  = IPS_RST_PIN,
                    bl_pin   = IPS_BL_PIN,
                    width    = IPS_WIDTH,
                    height   = IPS_HEIGHT,
                    spi_id   = IPS_SPI_ID,
                )
                print("[DISPLAY] IPS ready")
            except Exception as e:
                print("[DISPLAY] IPS failed:", e)

    def _ok(self):
        return self._oled is not None or self._ips is not None

    # ── PID status screen ─────────────────────────────────────
    def pid_screen(self, temp, sp, duty, kp, ki, kd, mode, safety, score,
                   tune_act, p_term=0., i_term=0., d_term=0.,
                   tune_count=0, loop_count=0):
        if not self._ok():  return

        if self._ips is not None:
            self._ips.update(
                temperature  = temp,
                setpoint     = sp,
                pid_output   = duty,
                p_term       = p_term,
                i_term       = i_term,
                d_term       = d_term,
                kp           = kp,
                ki           = ki,
                kd           = kd,
                safety_state = safety,
                tune_count   = tune_count,
                last_action  = mode[:3] + " " + tune_act,
                loop_count   = loop_count,
            )
            return

        # ── OLED ──────────────────────────────────────────────
        o = self._oled;  o.fill(0)
        err   = sp - (temp if temp is not None else sp)
        t_str = "{:.1f}C".format(temp) if temp else "---.-C"
        badge = self._BADGE.get(safety, "????")
        o.text("T:{} SP:{:.1f}".format(t_str, sp),  0,  0)
        o.text("Out:{:.0f}% [{}]".format(duty, badge),  0,  9)
        o.text("Err:{:+.2f}C".format(err),              0, 18)
        o.text("Kp:{:.2f} Ki:{:.4f}".format(kp, ki),    0, 27)
        o.text("Kd:{:.2f} {}".format(kd, mode[:3]),     0, 36)
        sc_str = "{:.3f}".format(score) if score < float('inf') else "---"
        o.text("score:{}".format(sc_str),                0, 45)
        o.text("SEL=menu",                               0, 56)
        o.show()

    # ── Menu screen ───────────────────────────────────────────
    def menu_screen(self, items, sel):
        """
        items: list of dicts with keys 'label', 'fmt'
        sel:   index of highlighted item
        """
        if not self._ok():  return

        if self._ips is not None:
            self._ips.menu_screen(items, sel)
            return

        # ── OLED ──────────────────────────────────────────────
        o = self._oled;  o.fill(0)
        o.text("--- SETTINGS ---",  0,  0)
        for i, item in enumerate(items):
            y    = 8 + i * 8
            line = "{} {}".format(item['label'], item['fmt'])
            line = (line + " " * 16)[:16]
            o.row(y, line, invert=(i == sel))
        o.text("SEL=next LP=save", 0, 56)
        o.show()


class MenuController:
    """
    Owns the menu item definitions and their edit logic.
    Separates concerns from the main loop.
    """

    def __init__(self, pid, state):
        self._pid   = pid
        self._state = state
        self._sel   = 0
        self._active = False
        self._items  = self._build()

    def _build(self):
        return [
            {'label': 'MODE', 'key': 'mode',
             'options': [StateManager.MODE_RUN, StateManager.MODE_CAL],
             'value': self._state.mode},
            {'label': 'TEMP', 'key': 'temp',
             'value': self._pid.setpoint, 'step': 0.5,
             'min': TEMP_MIN, 'max': TEMP_MAX},
            {'label': 'Kp  ', 'key': 'kp',
             'value': self._pid.kp, 'step': 0.1,
             'min': KP_MIN, 'max': KP_MAX},
            {'label': 'Ki  ', 'key': 'ki',
             'value': self._pid.ki, 'step': 0.005,
             'min': KI_MIN, 'max': KI_MAX},
            {'label': 'Kd  ', 'key': 'kd',
             'value': self._pid.kd, 'step': 0.1,
             'min': KD_MIN, 'max': KD_MAX},
        ]

    def _sync_from_live(self):
        """Refresh item values from current live state before opening menu."""
        self._items[0]['value'] = self._state.mode
        self._items[1]['value'] = self._pid.setpoint
        self._items[2]['value'] = self._pid.kp
        self._items[3]['value'] = self._pid.ki
        self._items[4]['value'] = self._pid.kd

    def _fmt(self, item):
        if item['key'] == 'mode':
            return item['value']
        if item['key'] == 'temp':
            return "{:.1f}C".format(item['value'])
        if item['key'] == 'ki':
            return "{:.4f}".format(item['value'])
        return "{:.3f}".format(item['value'])

    def formatted_items(self):
        return [{'label': it['label'], 'fmt': self._fmt(it)} for it in self._items]

    @property
    def active(self):
        return self._active

    @property
    def sel(self):
        return self._sel

    def open(self):
        self._sync_from_live()
        self._sel    = 0
        self._active = True
        print("[MENU] Opened")

    def close_and_apply(self):
        """Save menu values to PID and state, return new mode string."""
        it = {i['key']: i['value'] for i in self._items}
        self._pid.setpoint = it['temp']
        self._pid.load(it['kp'], it['ki'], it['kd'])
        self._active = False
        mode = it['mode']
        print("[MENU] Saved  TEMP={:.1f} Kp={:.3f} Ki={:.4f} Kd={:.3f} MODE={}".format(
            it['temp'], it['kp'], it['ki'], it['kd'], mode))
        return mode

    def handle(self, event):
        """Process a button event while menu is active. Returns True if menu closed."""
        if not self._active:
            return False

        item = self._items[self._sel]

        if event == 'sel':
            self._sel = (self._sel + 1) % len(self._items)

        elif event == 'sel_long':
            return True   # signal caller to close

        elif event in ('up', 'dn'):
            delta = 1 if event == 'up' else -1
            if item['key'] == 'mode':
                opts = item['options']
                idx  = opts.index(item['value'])
                item['value'] = opts[(idx + delta) % len(opts)]
            else:
                step  = item['step']
                item['value'] = round(
                    max(item['min'], min(item['max'],
                        item['value'] + delta * step)), 6)

        return False


# ============================================================
#  SECTION 12 — STATUS LED
# ============================================================

class StatusLED:
    def __init__(self, pin):
        self.led = Pin(pin, Pin.OUT);  self._n = 0;  self._s = 0

    def update(self, safety, active, mode, menu_open):
        self._n += 1
        if menu_open:
            # Solid ON while in menu so user knows they're editing
            self.led.value(1);  return
        if safety == SafetySystem.SHUTDOWN:
            self.led.value(0);  return
        if safety in (SafetySystem.WARNING, SafetySystem.CUTOFF):
            self.led.value(1);  return
        if mode == StateManager.MODE_CAL:
            pat = [1,0,1,0,0,0,0,0]
            self.led.value(pat[self._n % 8]);  return
        if active:
            self._s = 1 - self._s;  self.led.value(self._s)
        else:
            if self._n % 4 == 0:  self._s = 1 - self._s
            self.led.value(self._s)


# ============================================================
#  SECTION 13 — MAIN
# ============================================================

def main():
    print("=" * 52)
    print(" Pico PID v4  —  3-Button Menu UI")
    print("=" * 52)

    sensor  = TemperatureSensor(PIN_SENSOR)
    state   = StateManager()
    buttons = Buttons(PIN_SEL, PIN_UP, PIN_DN)
    safety  = SafetySystem(TARGET_TEMP)
    led     = StatusLED(PIN_LED)
    display = Display()

    has_best = state.best_score < float('inf')
    kp0 = state.best_kp if has_best else KP_DEFAULT
    ki0 = state.best_ki if has_best else KI_DEFAULT
    kd0 = state.best_kd if has_best else KD_DEFAULT

    pid    = PIDController(kp0, ki0, kd0, TARGET_TEMP)
    menu   = MenuController(pid, state)
    output = OutputDriver(PIN_OUTPUT, RELAY_MODE, PWM_FREQUENCY_HZ,
                          PWM_DUTY_MIN_PCT, PWM_DUTY_MAX_PCT)

    mode      = state.mode
    cal_start = None

    def enter_run():
        nonlocal mode, cal_start
        mode = StateManager.MODE_RUN;  cal_start = None
        state.set_mode(mode)
        pid.load(state.best_kp, state.best_ki, state.best_kd)
        tuner.reconfigure(RUN_TUNE_EVERY_S, RUN_TUNE_STEP)
        perf.reset()
        print("[RUN] Kp={:.3f} Ki={:.4f} Kd={:.3f}  best_score={:.3f}".format(
            pid.kp, pid.ki, pid.kd, state.best_score))

    def enter_calibrate():
        nonlocal mode, cal_start
        mode = StateManager.MODE_CAL;  cal_start = time.ticks_ms()
        state.set_mode(mode);  state.cal_count += 1
        pid.load(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT)
        tuner.reconfigure(CAL_TUNE_EVERY_S, CAL_TUNE_STEP)
        perf.reset()
        print("[CAL] Session #{} started".format(state.cal_count))

    tuner = AutoTuner(pid,
                      RUN_TUNE_EVERY_S if mode == StateManager.MODE_RUN else CAL_TUNE_EVERY_S,
                      RUN_TUNE_STEP    if mode == StateManager.MODE_RUN else CAL_TUNE_STEP)
    perf  = PerfTracker(RUN_TUNE_EVERY_S if mode == StateManager.MODE_RUN else CAL_TUNE_EVERY_S)

    if mode == StateManager.MODE_CAL:
        enter_calibrate()

    print(" Mode  :", mode)
    print(" Target:", TARGET_TEMP, "°C")
    print(" Buttons: GP17=SELECT  GP18=UP  GP19=DOWN")
    print(" Press SELECT to open settings menu")
    print("=" * 52)

    if SERIAL_PRINT:
        _log("ts_ms,temp_C,sp_C,err,duty,Kp,Ki,Kd,mode,safety,score")

    loop_count = 0
    last_loop  = time.ticks_ms()
    live_score = float('inf')

    while True:
        now     = time.ticks_ms()
        elapsed = time.ticks_diff(now, last_loop)
        if elapsed < LOOP_INTERVAL_MS:
            time.sleep_ms(LOOP_INTERVAL_MS - elapsed)
        last_loop = time.ticks_ms()
        loop_count += 1

        # ── Button events ─────────────────────────────────────
        for ev in buttons.read():

            if menu.active:
                # ── Inside menu ───────────────────────────────
                closed = menu.handle(ev)
                if closed:
                    new_mode = menu.close_and_apply()
                    if new_mode != mode:
                        if new_mode == StateManager.MODE_RUN:
                            enter_run()
                        else:
                            enter_calibrate()
                    else:
                        # Mode same — just re-sync tuner/perf with new gains
                        tuner.reconfigure(
                            RUN_TUNE_EVERY_S if mode == StateManager.MODE_RUN else CAL_TUNE_EVERY_S,
                            RUN_TUNE_STEP    if mode == StateManager.MODE_RUN else CAL_TUNE_STEP)
                        perf.reset()
            else:
                # ── Outside menu ──────────────────────────────
                if ev == 'sel':
                    menu.open()
                elif ev == 'sel_long':
                    # Long press outside menu = factory reset
                    print("[RESET] Long press — factory reset")
                    state.factory_reset()
                    enter_calibrate()

        # ── Auto-return from CALIBRATE ────────────────────────
        if mode == StateManager.MODE_CAL and cal_start is not None:
            if time.ticks_diff(time.ticks_ms(), cal_start)/1000. >= CAL_DURATION_S:
                print("[CAL] Complete — returning to RUN")
                enter_run()

        # ── Sensor + safety ───────────────────────────────────
        temperature = sensor.read_celsius()
        output_ok   = safety.check(temperature)

        if temperature is None:
            output.off();  output.tick()
            led.update(safety.state, False, mode, menu.active)
            if menu.active:
                display.menu_screen(menu.formatted_items(), menu.sel)
            continue

        # ── PID ───────────────────────────────────────────────
        pid_out = pid.compute(temperature)
        if output_ok:
            output.set(pid_out)
        else:
            output.off()
            if safety.state in (SafetySystem.CUTOFF, SafetySystem.SHUTDOWN):
                pid.reset()

        output.tick()

        # ── Performance tracking ──────────────────────────────
        error = pid.setpoint - temperature
        if not menu.active:
            tuner.record(temperature)
            perf.record(error)
            if perf.ready():
                live_score = perf.score()
                state.propose(pid.kp, pid.ki, pid.kd, live_score)
                if mode == StateManager.MODE_RUN and live_score > state.best_score * 1.3:
                    print("[RUN] Degraded — reloading best gains")
                    pid.load(state.best_kp, state.best_ki, state.best_kd)
                perf.reset()

        # ── LED ───────────────────────────────────────────────
        led.update(safety.state, output.duty > 0, mode, menu.active)

        # ── Display ───────────────────────────────────────────
        if loop_count % 4 == 0:
            if menu.active:
                display.menu_screen(menu.formatted_items(), menu.sel)
            else:
                display.pid_screen(temperature, pid.setpoint, output.duty,
                                   pid.kp, pid.ki, pid.kd, mode,
                                   safety.state, live_score, tuner.last,
                                   p_term=pid.last_p, i_term=pid.last_i,
                                   d_term=pid.last_d, tune_count=tuner.count,
                                   loop_count=loop_count)

        # ── Serial log ────────────────────────────────────────
        if loop_count % LOG_EVERY_LOOPS == 0:
            sc = "{:.3f}".format(live_score) if live_score < float('inf') else "---"
            print("T={:.2f}C Err={:+.2f} Out={:.0f}% Kp={:.3f} Ki={:.4f} Kd={:.3f}"
                  " [{}] {} score={}".format(
                temperature, error, output.duty,
                pid.kp, pid.ki, pid.kd, mode[:3], safety.state, sc))
            if SERIAL_PRINT:
                _log("{},{:.2f},{:.1f},{:.2f},{:.0f},{:.3f},{:.4f},{:.3f},{},{},{}".format(
                    time.ticks_ms(), temperature, pid.setpoint, error, output.duty,
                    pid.kp, pid.ki, pid.kd, mode[:3], safety.state, sc))


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
    Pin(PIN_OUTPUT, Pin.OUT).value(0)
