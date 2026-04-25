# ============================================================
#  Raspberry Pi Pico  —  Smart PID Controller  v3
#  MicroPython
# ============================================================
#
#  Two modes, selectable with a button:
#
#  RUN mode  (green LED pattern)
#    Loads the best-known PID gains from flash.
#    Runs gentle auto-tuning to keep improving.
#    Automatically saves gains to flash if performance gets better.
#
#  CALIBRATE mode  (rapid double-blink)
#    Resets to default gains and runs aggressive auto-tuning.
#    Designed to find good gains quickly when the system is new
#    or the environment has changed (new heater, new vessel, etc.)
#    Saves the best gains found, then you switch back to RUN.
#
#  Button (GP17):
#    Short press  (< 2 s) → toggle RUN ↔ CALIBRATE
#    Long press   (≥ 3 s) → factory reset  (clears saved gains,
#                            reloads defaults, enters CALIBRATE)
#
#  State saved to flash  →  pid_state.json
#  Contents: best Kp/Ki/Kd, best score, calibration count, uptime
#
# ============================================================
#  WIRING
# ============================================================
#
#  NTC thermistor (high side):
#    3.3V ──── NTC ──── GP26 (ADC0) ──── 100kΩ ──── GND
#
#  Output:
#    GP16 ──── relay / SSR / MOSFET
#
#  Built-in LED:
#    GP25  (use "LED" string for Pico W)
#
#  Button:
#    GP17 ──── push button ──── GND   (internal pull-up, no resistor needed)
#
#  OLED (set DISPLAY_TYPE = "OLED"):
#    GP14 ── SDA  |  GP15 ── SCL
#
#  IPS display (set DISPLAY_TYPE = "IPS"):
#    GP10 ── SCK    GP11 ── MOSI   GP13 ── CS
#    GP8  ── DC     GP9  ── RST    (BL optional)
#    Requires display_ips.py on the Pico filesystem.
#
# ============================================================


# ============================================================
#  SECTION 1 — SETTINGS
# ============================================================

TARGET_TEMP = 60.0

# ── Hardware pins ────────────────────────────────────────────
PIN_SENSOR  = 26
PIN_OUTPUT  = 16
PIN_LED     = 25
PIN_BUTTON  = 17       # push button to GND, internal pull-up used

# ── NTC thermistor ───────────────────────────────────────────
NTC_R_SERIES = 100_000
NTC_R_AT_25C = 100_000
NTC_B_VALUE  = 3950
ADC_VOLTAGE  = 3.3

# ── Output ───────────────────────────────────────────────────
RELAY_MODE        = False
PWM_FREQUENCY_HZ  = 1
PWM_DUTY_MIN_PCT  = 0
PWM_DUTY_MAX_PCT  = 100

# ── Safety ───────────────────────────────────────────────────
SAFETY_HARD_MAX_C = 90.0
SAFETY_CUTOFF_PCT = 20.0
SAFETY_WARN_PCT   = 10.0

# ── Default (factory) PID gains ──────────────────────────────
# These are used when no saved state exists, or after factory reset.
KP_DEFAULT = 5.0
KI_DEFAULT = 0.05
KD_DEFAULT = 1.0

# ── PID gain limits ──────────────────────────────────────────
KP_MIN, KP_MAX = 0.5, 50.0
KI_MIN, KI_MAX = 0.0,  5.0
KD_MIN, KD_MAX = 0.0, 10.0

# ── RUN mode — gentle tuning ──────────────────────────────────
# Applies small corrections to keep improving over time.
RUN_TUNE_EVERY_S  = 60      # tune every 60 s
RUN_TUNE_STEP     = 0.03    # nudge gains by ±3 % per cycle

# ── CALIBRATE mode — aggressive tuning ───────────────────────
# Finds good gains faster.  Use when starting fresh.
CAL_TUNE_EVERY_S  = 20      # tune every 20 s
CAL_TUNE_STEP     = 0.10    # nudge gains by ±10 % per cycle
CAL_DURATION_S    = 300     # auto-return to RUN after 5 minutes

# ── Performance scoring ───────────────────────────────────────
# Score = mean absolute error + oscillation penalty.
# Lower score = better performance.
# Saved gains are updated only when score improves.
SCORE_OZONE_DEADBAND = 1.0  # °C — errors smaller than this count as "perfect"

# ── Flash storage ─────────────────────────────────────────────
STATE_FILE = "pid_state.json"

# ── Loop timing ───────────────────────────────────────────────
LOOP_INTERVAL_MS  = 500
LOG_EVERY_LOOPS   = 10

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

DISPLAY_UPDATE_EVERY = 4

# ── Serial ────────────────────────────────────────────────────
SERIAL_PRINT = True


# ============================================================
#  SECTION 2 — IMPORTS
# ============================================================

from machine import Pin, ADC
import time, math, json, sys


def _log(line):
    if SERIAL_PRINT:
        sys.stdout.write(line + "\r\n")


_display_obj = None
if DISPLAY_TYPE == "OLED":
    try:
        from display_oled import OLEDDisplay as _OLEDDisplay
        _display_obj = _OLEDDisplay(
            sda_pin  = OLED_SDA_PIN,
            scl_pin  = OLED_SCL_PIN,
            i2c_addr = OLED_I2C_ADDR,
        )
        print("[DISPLAY] OLED ready")
    except Exception as e:
        print("[DISPLAY] OLED failed:", e)
elif DISPLAY_TYPE == "IPS":
    try:
        from display_ips import IPSDisplay as _IPSDisplay
        _display_obj = _IPSDisplay(
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


# ============================================================
#  SECTION 3 — TEMPERATURE SENSOR
# ============================================================

class TemperatureSensor:
    _ADC_MAX = 65535

    def __init__(self, pin_number):
        self.adc = ADC(Pin(pin_number))

    def read_celsius(self):
        try:
            raw = self.adc.read_u16()
            if raw <= 10 or raw >= self._ADC_MAX - 10:
                return None
            voltage  = (raw / self._ADC_MAX) * ADC_VOLTAGE
            r_ntc    = NTC_R_SERIES * (ADC_VOLTAGE - voltage) / voltage
            t_kelvin = 1.0 / (1.0 / 298.15 + math.log(r_ntc / NTC_R_AT_25C) / NTC_B_VALUE)
            return t_kelvin - 273.15
        except Exception as e:
            print("[SENSOR]", e)
            return None


# ============================================================
#  SECTION 4 — PID CONTROLLER
# ============================================================

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp;  self.ki = ki;  self.kd = kd
        self.setpoint    = setpoint
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_time  = None
        self.last_p = self.last_i = self.last_d = 0.0

    def compute(self, measured):
        now = time.ticks_ms()
        dt  = (LOOP_INTERVAL_MS / 1000.0) if self._last_time is None else \
              max(0.001, time.ticks_diff(now, self._last_time) / 1000.0)
        self._last_time = now

        error  = self.setpoint - measured
        p_term = self.kp * error

        self._integral += error * dt
        if self.ki > 0.0001:
            cap = 100.0 / self.ki
            self._integral = max(-cap, min(cap, self._integral))
        i_term = self.ki * self._integral

        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term

        return max(0.0, min(100.0, p_term + i_term + d_term))

    def reset(self):
        self._integral = 0.0;  self._prev_error = 0.0;  self._last_time = None

    def load_gains(self, kp, ki, kd):
        self.kp = kp;  self.ki = ki;  self.kd = kd
        self.reset()


# ============================================================
#  SECTION 5 — OUTPUT DRIVER  (soft-PWM)
# ============================================================

class OutputDriver:
    def __init__(self, pin_number, relay_mode, freq_hz, duty_min, duty_max):
        self.relay_mode   = relay_mode
        self.duty_min     = duty_min
        self.duty_max     = duty_max
        self.current_duty = 0.0
        self.pin          = Pin(pin_number, Pin.OUT)
        self.pin.value(0)
        if not relay_mode:
            self.period_ms    = int(1000 / freq_hz)
            self._cycle_start = time.ticks_ms()

    def set_output(self, pct):
        duty = 0.0 if pct <= 0.5 else max(self.duty_min, min(self.duty_max, pct))
        self.current_duty = duty
        if self.relay_mode:
            self.pin.value(1 if duty > 50.0 else 0)

    def tick(self):
        if self.relay_mode:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self._cycle_start)
        if elapsed >= self.period_ms:
            self._cycle_start = time.ticks_ms();  elapsed = 0
        on_ms = int(self.current_duty / 100.0 * self.period_ms)
        self.pin.value(1 if (on_ms > 0 and elapsed < on_ms) else 0)

    def turn_off(self):
        self.current_duty = 0.0;  self.pin.value(0)


# ============================================================
#  SECTION 6 — SAFETY SYSTEM
# ============================================================

class SafetySystem:
    OK = "OK";  WARNING = "WARNING";  CUTOFF = "CUTOFF";  SHUTDOWN = "SHUTDOWN"

    def __init__(self, setpoint):
        self.warn_c   = setpoint * (1 + SAFETY_WARN_PCT   / 100.0)
        self.cutoff_c = setpoint * (1 + SAFETY_CUTOFF_PCT / 100.0)
        self.hard_max = SAFETY_HARD_MAX_C
        self.state    = self.OK
        self.blocked  = False
        print("[SAFETY] Warn:{:.1f}  Cutoff:{:.1f}  Max:{:.1f}".format(
            self.warn_c, self.cutoff_c, self.hard_max))

    def check(self, t):
        if t is None:
            self.state = self.WARNING;  self.blocked = True;  return False
        if t >= self.hard_max:
            if self.state != self.SHUTDOWN:
                print("[SAFETY] SHUTDOWN {:.1f}°C".format(t))
            self.state = self.SHUTDOWN;  self.blocked = True;  return False
        if t >= self.cutoff_c:
            if self.state not in (self.CUTOFF, self.SHUTDOWN):
                print("[SAFETY] CUTOFF {:.1f}°C".format(t))
            self.state = self.CUTOFF;  self.blocked = True;  return False
        if t >= self.warn_c:
            self.state = self.WARNING;  self.blocked = False;  return True
        if self.state == self.CUTOFF:
            print("[SAFETY] Resumed {:.1f}°C".format(t))
        self.state = self.OK;  self.blocked = False;  return True


# ============================================================
#  SECTION 7 — BUTTON HANDLER
# ============================================================

class Button:
    """
    Debounced button on a pin wired to GND.
    Internal pull-up keeps pin HIGH when not pressed.

    Returns on check():
      'short'  — pressed and released in < 2 s   → switch mode
      'long'   — held for ≥ 3 s                   → factory reset
      None     — nothing happened
    """
    SHORT_MAX_MS = 2000
    LONG_MIN_MS  = 3000
    DEBOUNCE_MS  = 50

    def __init__(self, pin_number):
        self.pin         = Pin(pin_number, Pin.IN, Pin.PULL_UP)
        self._pressed    = False
        self._press_time = 0
        self._last_raw   = 1
        self._last_ms    = 0

    def check(self):
        now = time.ticks_ms()
        raw = self.pin.value()   # 0 = pressed (pulled to GND)

        # Debounce: ignore changes faster than DEBOUNCE_MS
        if raw == self._last_raw:
            pass
        elif time.ticks_diff(now, self._last_ms) >= self.DEBOUNCE_MS:
            self._last_raw = raw
            self._last_ms  = now

            if raw == 0 and not self._pressed:   # press start
                self._pressed    = True
                self._press_time = now

            elif raw == 1 and self._pressed:     # released
                self._pressed  = False
                held = time.ticks_diff(now, self._press_time)
                if held >= self.LONG_MIN_MS:
                    return 'long'
                elif held >= self.DEBOUNCE_MS:
                    return 'short'

        # Detect long press while still held (before release)
        if self._pressed:
            held = time.ticks_diff(now, self._press_time)
            if held >= self.LONG_MIN_MS:
                self._pressed = False   # consume event
                return 'long'

        return None


# ============================================================
#  SECTION 8 — STATE MANAGER
# ============================================================

class StateManager:
    """
    Saves and loads the best-known PID gains to flash.

    best_score tracks performance — lower is better.
    Gains are only overwritten when a new score beats the saved best.

    Also tracks:
      mode          — "RUN" or "CALIBRATE" (restored on reboot)
      cal_count     — how many calibration sessions have run
      total_loops   — lifetime loop count for diagnostics
    """

    MODE_RUN = "RUN"
    MODE_CAL = "CALIBRATE"

    def __init__(self):
        self.best_kp    = KP_DEFAULT
        self.best_ki    = KI_DEFAULT
        self.best_kd    = KD_DEFAULT
        self.best_score = float('inf')   # lower = better; inf means no data yet
        self.mode       = self.MODE_RUN
        self.cal_count  = 0
        self.total_loops = 0
        self._load()

    # ── Candidate update ─────────────────────────────────────
    def propose(self, kp, ki, kd, score):
        """
        Compare score to current best.
        If better, save new gains to flash and return True.
        """
        if score < self.best_score:
            self.best_kp    = kp
            self.best_ki    = ki
            self.best_kd    = kd
            self.best_score = score
            self._save()
            print("[STATE] New best!  score={:.3f}  Kp={:.3f}  Ki={:.4f}  Kd={:.3f}".format(
                score, kp, ki, kd))
            return True
        return False

    def set_mode(self, mode):
        self.mode = mode
        self._save()
        print("[MODE] →", mode)

    def factory_reset(self):
        self.best_kp    = KP_DEFAULT
        self.best_ki    = KI_DEFAULT
        self.best_kd    = KD_DEFAULT
        self.best_score = float('inf')
        self.mode       = self.MODE_CAL
        self._save()
        print("[STATE] Factory reset — all gains cleared")

    def increment_cal(self):
        self.cal_count += 1;  self._save()

    # ── Persistence ──────────────────────────────────────────
    def _save(self):
        try:
            with open(STATE_FILE, 'w') as f:
                json.dump({
                    'best_kp':    self.best_kp,
                    'best_ki':    self.best_ki,
                    'best_kd':    self.best_kd,
                    'best_score': self.best_score,
                    'mode':       self.mode,
                    'cal_count':  self.cal_count,
                }, f)
        except Exception as e:
            print("[STATE] Save failed:", e)

    def _load(self):
        try:
            with open(STATE_FILE, 'r') as f:
                d = json.load(f)
            self.best_kp    = d.get('best_kp',    KP_DEFAULT)
            self.best_ki    = d.get('best_ki',    KI_DEFAULT)
            self.best_kd    = d.get('best_kd',    KD_DEFAULT)
            self.best_score = d.get('best_score', float('inf'))
            self.mode       = d.get('mode',       self.MODE_RUN)
            self.cal_count  = d.get('cal_count',  0)
            if self.best_score == float('inf'):
                print("[STATE] No saved gains — starting fresh")
            else:
                print("[STATE] Loaded  score={:.3f}  Kp={:.3f}  Ki={:.4f}  Kd={:.3f}".format(
                    self.best_score, self.best_kp, self.best_ki, self.best_kd))
        except:
            print("[STATE] No state file — using defaults")


# ============================================================
#  SECTION 9 — PERFORMANCE TRACKER
# ============================================================

class PerformanceTracker:
    """
    Accumulates error samples over a tune window and produces a score.

    score = mean_absolute_error + oscillation_penalty
      - mean_absolute_error: average |error| over the window
      - oscillation_penalty: 0.5 × sign_changes_per_minute

    Lower score = better control.
    Errors smaller than SCORE_OZONE_DEADBAND are treated as 0
    so small steady-state noise does not unfairly penalise good gains.
    """

    def __init__(self, window_s):
        self.window_s     = window_s
        self._errors      = []
        self._sign_changes = 0
        self._prev_sign   = 0
        self._start       = time.ticks_ms()

    def record(self, error):
        ae = abs(error)
        self._errors.append(max(0.0, ae - SCORE_OZONE_DEADBAND))

        sign = 1 if error > 0 else (-1 if error < 0 else 0)
        if sign != 0 and self._prev_sign != 0 and sign != self._prev_sign:
            self._sign_changes += 1
        self._prev_sign = sign

    def ready(self):
        elapsed = time.ticks_diff(time.ticks_ms(), self._start) / 1000.0
        return elapsed >= self.window_s and len(self._errors) >= 5

    def score(self):
        if not self._errors:
            return float('inf')
        n       = len(self._errors)
        mae     = sum(self._errors) / n
        elapsed_min = self.window_s / 60.0
        osc_rate    = self._sign_changes / max(elapsed_min, 0.01)
        return mae + 0.5 * osc_rate

    def reset(self):
        self._errors       = []
        self._sign_changes = 0
        self._prev_sign    = 0
        self._start        = time.ticks_ms()


# ============================================================
#  SECTION 10 — AUTO-TUNER  (mode-aware)
# ============================================================

class AutoTuner:
    """
    Same rule-based tuner as v1, but step size and interval
    are passed in at construction so they can differ between
    RUN mode and CALIBRATE mode.
    """

    def __init__(self, pid, tune_every_s, step_size):
        self.pid         = pid
        self.tune_every  = tune_every_s
        self.step        = step_size
        self._errors     = []
        self._sign_changes = 0
        self._prev_sign  = 0
        self._last_tune  = time.ticks_ms()
        self.tune_count  = 0
        self.last_action = "none"

    def record(self, temperature):
        error = self.pid.setpoint - temperature
        self._errors.append(error)
        sign = 1 if error > 0 else (-1 if error < 0 else 0)
        if sign != 0 and self._prev_sign != 0 and sign != self._prev_sign:
            self._sign_changes += 1
        self._prev_sign = sign
        elapsed = time.ticks_diff(time.ticks_ms(), self._last_tune) / 1000.0
        if elapsed >= self.tune_every:
            self._tune(elapsed)

    def _tune(self, elapsed_s):
        if len(self._errors) < 5:
            return
        n            = len(self._errors)
        avg_err      = sum(self._errors) / n
        avg_abs      = sum(abs(e) for e in self._errors) / n
        osc          = self._sign_changes / max(elapsed_s / 60.0, 0.01)
        actions      = []

        if osc > 4:
            self.pid.kp = max(KP_MIN, min(KP_MAX, self.pid.kp * (1 - self.step)))
            self.pid.kd = max(KD_MIN, min(KD_MAX, self.pid.kd * (1 + self.step)))
            actions.append("osc({:.1f}/min) Kp- Kd+".format(osc))
        elif abs(avg_err) > 1.0:
            self.pid.ki = max(KI_MIN, min(KI_MAX, self.pid.ki * (1 + self.step)))
            actions.append("offset({:.2f}C) Ki+".format(avg_err))
        elif avg_abs > 5.0 and osc < 1:
            self.pid.kp = max(KP_MIN, min(KP_MAX, self.pid.kp * (1 + self.step * 0.5)))
            actions.append("slow({:.2f}C) Kp+".format(avg_abs))
        else:
            actions.append("ok({:.2f}C)".format(avg_abs))

        self.last_action = " | ".join(actions)
        self.tune_count += 1
        print("[TUNE#{}] {}  Kp={:.3f} Ki={:.4f} Kd={:.3f}".format(
            self.tune_count, self.last_action,
            self.pid.kp, self.pid.ki, self.pid.kd))

        self._errors       = []
        self._sign_changes = 0
        self._last_tune    = time.ticks_ms()

    def reconfigure(self, tune_every_s, step_size):
        """Call when switching modes to update tuner parameters."""
        self.tune_every = tune_every_s
        self.step       = step_size
        self._errors    = []
        self._last_tune = time.ticks_ms()


# ============================================================
#  SECTION 11 — STATUS LED  (mode-aware)
# ============================================================

class StatusLED:
    """
    RUN mode       — existing behaviour (fast/slow blink)
    CALIBRATE mode — double-blink pattern every 2 s
    RESET pending  — rapid solid flash
    """

    def __init__(self, pin_number):
        self.led      = Pin(pin_number, Pin.OUT)
        self._cnt     = 0
        self._state   = 0

    def update(self, safety_state, output_active, mode):
        self._cnt += 1

        if safety_state == SafetySystem.SHUTDOWN:
            self.led.value(0);  return

        if safety_state in (SafetySystem.WARNING, SafetySystem.CUTOFF):
            self.led.value(1);  return

        if mode == StateManager.MODE_CAL:
            # Double-blink: ON-off-ON-off-off-off-off-off (8 loop cycle)
            pat = [1, 0, 1, 0, 0, 0, 0, 0]
            self.led.value(pat[self._cnt % 8])
            return

        # RUN mode — standard behaviour
        if output_active:
            self._state = 1 - self._state
            self.led.value(self._state)
        else:
            if self._cnt % 4 == 0:
                self._state = 1 - self._state
            self.led.value(self._state)


# ============================================================
#  SECTION 12 — MAIN
# ============================================================

def main():
    print("=" * 52)
    print(" Pico Smart PID  v3  —  Starting up")
    print("=" * 52)

    # ── Create objects ────────────────────────────────────────
    sensor  = TemperatureSensor(PIN_SENSOR)
    state   = StateManager()
    button  = Button(PIN_BUTTON)
    safety  = SafetySystem(TARGET_TEMP)

    # Load gains: best saved if available, otherwise defaults
    kp0 = state.best_kp if state.best_score < float('inf') else KP_DEFAULT
    ki0 = state.best_ki if state.best_score < float('inf') else KI_DEFAULT
    kd0 = state.best_kd if state.best_score < float('inf') else KD_DEFAULT

    pid = PIDController(kp=kp0, ki=ki0, kd=kd0, setpoint=TARGET_TEMP)

    output = OutputDriver(
        pin_number = PIN_OUTPUT,
        relay_mode = RELAY_MODE,
        freq_hz    = PWM_FREQUENCY_HZ,
        duty_min   = PWM_DUTY_MIN_PCT,
        duty_max   = PWM_DUTY_MAX_PCT,
    )

    # Tuner starts with RUN mode parameters
    tuner = AutoTuner(
        pid          = pid,
        tune_every_s = RUN_TUNE_EVERY_S,
        step_size    = RUN_TUNE_STEP,
    )

    # Performance tracker window matches the tuner interval
    perf = PerformanceTracker(window_s=RUN_TUNE_EVERY_S)

    led = StatusLED(PIN_LED)

    display = _display_obj

    # ── Mode setup ────────────────────────────────────────────
    mode = state.mode
    cal_start = None     # when current calibration session started

    def enter_run():
        nonlocal mode, cal_start
        mode      = StateManager.MODE_RUN
        cal_start = None
        state.set_mode(mode)
        # Switch to best known gains
        pid.load_gains(state.best_kp, state.best_ki, state.best_kd)
        tuner.reconfigure(RUN_TUNE_EVERY_S, RUN_TUNE_STEP)
        perf.reset()
        print("[RUN] Loaded gains  Kp={:.3f} Ki={:.4f} Kd={:.3f}".format(
            pid.kp, pid.ki, pid.kd))
        print("[RUN] Best score so far: {:.3f}".format(state.best_score))

    def enter_calibrate():
        nonlocal mode, cal_start
        mode      = StateManager.MODE_CAL
        cal_start = time.ticks_ms()
        state.set_mode(mode)
        state.increment_cal()
        # Reset to defaults for a clean calibration
        pid.load_gains(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT)
        tuner.reconfigure(CAL_TUNE_EVERY_S, CAL_TUNE_STEP)
        perf.reset()
        print("[CALIBRATE] Gains reset to defaults for session #{}".format(state.cal_count))
        print("[CALIBRATE] Will auto-return to RUN after {} s".format(CAL_DURATION_S))

    # Apply the mode that was saved in flash
    if mode == StateManager.MODE_CAL:
        enter_calibrate()
    else:
        enter_run()

    print(" Mode   :", mode)
    print(" Target :", TARGET_TEMP, "°C")
    print(" Button : short press = toggle mode | long press = factory reset")
    print("=" * 52)

    if SERIAL_PRINT:
        _log("ts_ms,temp_C,setpoint_C,error,duty_pct,Kp,Ki,Kd,mode,safety,score")

    loop_count = 0
    last_loop  = time.ticks_ms()

    while True:

        # ── Timing ───────────────────────────────────────────
        now     = time.ticks_ms()
        elapsed = time.ticks_diff(now, last_loop)
        if elapsed < LOOP_INTERVAL_MS:
            time.sleep_ms(LOOP_INTERVAL_MS - elapsed)
        last_loop = time.ticks_ms()
        loop_count += 1

        # ── Button ───────────────────────────────────────────
        btn = button.check()
        if btn == 'short':
            if mode == StateManager.MODE_RUN:
                enter_calibrate()
            else:
                enter_run()
        elif btn == 'long':
            print("[RESET] Factory reset triggered by long press")
            state.factory_reset()
            enter_calibrate()

        # ── Auto-return from CALIBRATE after timeout ──────────
        if mode == StateManager.MODE_CAL and cal_start is not None:
            cal_elapsed = time.ticks_diff(time.ticks_ms(), cal_start) / 1000.0
            if cal_elapsed >= CAL_DURATION_S:
                print("[CALIBRATE] Session complete ({} s) — returning to RUN".format(CAL_DURATION_S))
                enter_run()

        # ── Read sensor ───────────────────────────────────────
        temperature = sensor.read_celsius()
        output_allowed = safety.check(temperature)

        if temperature is None:
            output.turn_off()
            output.tick()
            led.update(safety.state, False, mode)
            continue

        # ── PID compute ───────────────────────────────────────
        pid_output = pid.compute(temperature)

        # ── Apply output ──────────────────────────────────────
        if output_allowed:
            output.set_output(pid_output)
            is_active = output.current_duty > 0
        else:
            output.turn_off()
            is_active = False
            if safety.state in (SafetySystem.CUTOFF, SafetySystem.SHUTDOWN):
                pid.reset()

        output.tick()

        # ── Track performance ─────────────────────────────────
        error = TARGET_TEMP - temperature
        tuner.record(temperature)
        perf.record(error)

        # When a scoring window completes, check if gains improved
        if perf.ready():
            current_score = perf.score()
            improved = state.propose(pid.kp, pid.ki, pid.kd, current_score)
            if improved:
                print("[PERF] Score improved: {:.3f} → {:.3f}".format(
                    state.best_score, current_score))
            else:
                print("[PERF] Score {:.3f}  (best {:.3f})".format(
                    current_score, state.best_score))

            # In RUN mode: if current gains have drifted worse, reload best
            if mode == StateManager.MODE_RUN and not improved:
                if current_score > state.best_score * 1.3:   # 30% worse
                    print("[RUN] Performance degraded — reloading best gains")
                    pid.load_gains(state.best_kp, state.best_ki, state.best_kd)
                    tuner.reconfigure(RUN_TUNE_EVERY_S, RUN_TUNE_STEP)

            perf.reset()

        # ── LED ───────────────────────────────────────────────
        led.update(safety.state, is_active, mode)

        # ── Display ───────────────────────────────────────────
        if display is not None and loop_count % DISPLAY_UPDATE_EVERY == 0:
            try:
                display.update(
                    temperature  = temperature,
                    setpoint     = TARGET_TEMP,
                    pid_output   = output.current_duty,
                    p_term       = pid.last_p,
                    i_term       = pid.last_i,
                    d_term       = pid.last_d,
                    kp           = pid.kp,
                    ki           = pid.ki,
                    kd           = pid.kd,
                    safety_state = safety.state,
                    tune_count   = tuner.tune_count,
                    last_action  = mode + " " + tuner.last_action,
                    loop_count   = loop_count,
                )
            except Exception as e:
                print("[DISPLAY] update error:", e)

        # ── Serial log ────────────────────────────────────────
        if loop_count % LOG_EVERY_LOOPS == 0:
            score_str = "{:.3f}".format(perf.score()) if perf._errors else "---"
            print("T={:.2f}C  SP={:.1f}C  Err={:+.2f}  Out={:.0f}%"
                  "  Kp={:.3f} Ki={:.4f} Kd={:.3f}"
                  "  [{}]  Safety={}  score={}".format(
                temperature, TARGET_TEMP, error, output.current_duty,
                pid.kp, pid.ki, pid.kd,
                mode[:3], safety.state, score_str))
            if SERIAL_PRINT:
                _log("{},{:.2f},{:.1f},{:.2f},{:.0f},{:.3f},{:.4f},{:.3f},{},{},{}".format(
                    time.ticks_ms(), temperature, TARGET_TEMP, error,
                    output.current_duty, pid.kp, pid.ki, pid.kd,
                    mode[:3], safety.state, score_str))


try:
    main()
except KeyboardInterrupt:
    print("\nStopped. Output off.")
    Pin(PIN_OUTPUT, Pin.OUT).value(0)
