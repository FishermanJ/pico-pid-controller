# ============================================================
#  ESP32 PID Controller  —  MicroPython
# ============================================================
#
#  What this program does:
#    1. Reads temperature from a sensor every loop
#    2. PID controller calculates how much power to give
#    3. PWM signal switches a relay / SSR / MOSFET
#    4. Auto-Tuner watches performance and slowly improves PID
#    5. Safety system shuts down if temperature gets dangerous
#
#  Good for controlling:
#    • Heaters  • Ovens  • Water baths  • Fermentation chambers
#    • Any process where you want to hold a stable temperature
#
# ============================================================
#  WIRING
# ============================================================
#
#  NTC Thermistor (temperature sensor):
#    3.3V ──── NTC ──── GPIO34 ──── 10kΩ ──── GND
#                           ↑ read voltage here
#
#  DS18B20 (alternative digital sensor):
#    3.3V ──── DS18B20 VCC
#    GND  ──── DS18B20 GND
#    GPIO33 ── DS18B20 DATA ──── 4.7kΩ ──── 3.3V   (pull-up!)
#
#  Relay / SSR / MOSFET:
#    GPIO25 ──── relay IN  (controls your heater)
#
#  Status LED (optional):
#    GPIO2 = built-in LED on most ESP32 boards
#
# ============================================================


# ============================================================
#  SECTION 1 — SETTINGS
#  This is the only section you need to edit!
# ============================================================

# ── What temperature do you want to reach? ──────────────────
TARGET_TEMP = 60.0          # Target in °C

# ── PID Gains (starting values — Auto-Tuner will improve them)
# Kp  →  How fast to react.   High = fast but can overshoot.
# Ki  →  Fix steady-state offset.  High = removes offset but can oscillate.
# Kd  →  Smooth out oscillations.  High = slower, more stable.
KP_START = 5.0
KI_START = 0.05
KD_START = 1.0

# ── PWM output settings ─────────────────────────────────────
PWM_FREQUENCY_HZ = 1        # Hz.  1 Hz works well for slow thermal systems.
                             # Use 1000+ Hz for motors / fast systems.
PWM_DUTY_MIN_PCT = 0        # % — minimum duty cycle when output is active
PWM_DUTY_MAX_PCT = 100      # % — maximum duty cycle (hard ceiling for safety)

# ── Control mode ─────────────────────────────────────────────
# True  = Digital relay  (just ON or OFF — simple mechanical relay)
# False = PWM output     (smooth control — use with SSR or MOSFET)
RELAY_MODE = False

# ── Sensor type ──────────────────────────────────────────────
SENSOR_TYPE = "NTC"         # "NTC"    = cheap thermistor (needs calibration)
                             # "DS18B20" = digital sensor (easier, more accurate)

# ── GPIO pin numbers ─────────────────────────────────────────
PIN_SENSOR  = 26            # ADC pin for NTC  /  data pin for DS18B20
PIN_OUTPUT  = 16            # Relay or SSR/MOSFET control signal
PIN_LED     = 25             # Status LED  (2 = built-in on most ESP32)

# ── NTC thermistor calibration (only needed if SENSOR_TYPE = "NTC") ─────────
NTC_R_PULLUP  = 10_000      # Ω  — the fixed resistor you put to GND
NTC_R_AT_25C  = 10_000      # Ω  — NTC resistance at exactly 25 °C
NTC_B_VALUE   = 3950        # B-coefficient from your NTC datasheet (usually 3950)
ADC_MAX       = 4095        # Leave as 4095 — overridden automatically for Pico (65535)
ADC_VOLTAGE   = 3.3         # Volts — reference voltage (3.3 V on both ESP32 and Pico)

# ── Safety limits ────────────────────────────────────────────
SAFETY_HARD_MAX_C   = 90.0  # °C  — ABSOLUTE LIMIT. Output shuts off completely.
SAFETY_WARN_PCT     = 10.0  # %   — Warning zone. e.g. 10% above 60°C = 66°C
SAFETY_CUTOFF_PCT   = 20.0  # %   — Emergency cutoff. e.g. 20% above 60°C = 72°C

# ── Auto-Tuner settings (the learning / ML part) ─────────────
AUTOTUNER_ENABLED   = True  # Turn learning on or off
TUNE_EVERY_SECONDS  = 30    # Check performance and nudge gains every 30 s
TUNE_STEP_SIZE      = 0.05  # How aggressively gains are changed each tune (5%)
                             # Smaller = safer but slower learning
KP_MIN, KP_MAX = 0.5, 50.0 # Allowed range for Kp
KI_MIN, KI_MAX = 0.0,  5.0 # Allowed range for Ki
KD_MIN, KD_MAX = 0.0, 10.0 # Allowed range for Kd

# ── Loop timing ──────────────────────────────────────────────
LOOP_INTERVAL_MS  = 500     # Run control loop every 500 ms (0.5 seconds)
LOG_EVERY_LOOPS   = 10      # Print status every N loops (10 × 0.5s = every 5 s)

# ── Display settings ─────────────────────────────────────────
# Choose ONE of:  "OLED"  |  "IPS"  |  "NONE"
DISPLAY_TYPE = "OLED"

# OLED — SSD1306 128×64 via I2C
#   Cheap monochrome display, very common for ESP32 projects.
#   Uses 2 wires (SDA + SCL).  Built-in MicroPython driver.
OLED_SDA_PIN   = 14          # SDA data wire
OLED_SCL_PIN   = 15          # SCL clock wire
OLED_I2C_ADDR  = 0x3C        # I2C address  (try 0x3D if display not found)

# IPS — ST7789 240×240 or 240×320 color display via SPI
#   Full color, shows temperature graph, larger text.
#   Uses 5 wires (SCK, MOSI, CS, DC, RST) + optional backlight.
IPS_SCK_PIN    = 18           # SPI clock
IPS_MOSI_PIN   = 23           # SPI data (MOSI)
IPS_CS_PIN     = 5            # Chip select
IPS_DC_PIN     = 16           # Data / Command select
IPS_RST_PIN    = 17           # Hardware reset
IPS_BL_PIN     = -1           # Backlight control (-1 = always on / not wired)
IPS_WIDTH      = 240          # Screen width  in pixels
IPS_HEIGHT     = 240          # Screen height in pixels  (240 or 320)
IPS_X_OFFSET   = 0            # Some 240×240 modules need x_offset=0, y_offset=80
IPS_Y_OFFSET   = 0

# How often to refresh the display (every N control loops)
# 1 = every loop (500 ms),  2 = every 1 s,  4 = every 2 s
# Increase if the display update makes the loop too slow.
DISPLAY_UPDATE_EVERY = 2

# ── Serial data output ────────────────────────────────────────
# True  = structured PID data is sent over Serial every loop
# False = no serial data output  (all other print/debug still works)
SERIAL_PRINT        = True      # ← set False to disable completely
SERIAL_BAUD         = 115200    # Baud rate — must match your Serial Monitor / Plotter
SERIAL_FORMAT       = "CSV"     # "CSV"   → comma-separated, works with Serial Plotter
                                 # "HUMAN" → labeled values, easy to read in a terminal
SERIAL_EVERY_LOOPS  = 1         # Send data every N loops  (1 = every 500 ms)


# ============================================================
#  SECTION 2 — IMPORTS
#  Load built-in MicroPython libraries
# ============================================================

from machine import Pin, PWM, ADC
import time
import math
import sys as _sys

# ── Platform detection ────────────────────────────────────────
# Detects whether we are running on a Raspberry Pi Pico (RP2040)
# or an ESP32 so the ADC is initialised correctly for each board.
#   ESP32  → sys.platform == "esp32"  | 12-bit ADC (0–4095)  | needs ATTN_11DB
#   Pico   → sys.platform == "rp2"   | 16-bit ADC (0–65535) | no attenuation
_IS_PICO  = _sys.platform == "rp2"
_IS_ESP32 = _sys.platform == "esp32"
_ADC_MAX  = 65535 if _IS_PICO else 4095   # overrides the ADC_MAX constant above
print(f"[BOARD] Platform: {_sys.platform} | ADC max: {_ADC_MAX}")

# ── Serial output initialisation ─────────────────────────────
# MicroPython's sys.stdout already points to UART0 (the USB-serial
# port used by the REPL).  We write directly to it — no need to
# create a new UART object (doing so would raise "bad TX pin").
# Set SERIAL_BAUD in your Serial Monitor to match the board default
# (usually 115200).  Changing baud rate at runtime is not supported
# here because UART0 is managed by the REPL.
if SERIAL_PRINT:
    import sys as _sys

def _serial_write(line):
    """Send one line of text to sys.stdout (UART0) when serial is active."""
    if SERIAL_PRINT:
        _sys.stdout.write(line + "\r\n")

# ── Display driver import (only loads the file you chose) ────
# Wrapped in try/except so a missing display never crashes startup.
_display_class = None
if DISPLAY_TYPE == "OLED":
    try:
        from display_oled import OLEDDisplay as _display_class
    except ImportError:
        print("[DISPLAY] display_oled.py not found — running without display")
elif DISPLAY_TYPE == "IPS":
    try:
        from display_ips import IPSDisplay as _display_class
    except ImportError:
        print("[DISPLAY] display_ips.py not found — running without display")


# ============================================================
#  SECTION 3 — TEMPERATURE SENSOR
# ============================================================

class TemperatureSensor:
    """
    Reads temperature from either an NTC thermistor or DS18B20 sensor.

    NTC thermistor:
      - Cheap and common.  Connected in a voltage divider with a fixed resistor.
      - We measure the voltage, calculate resistance, then convert to °C.

    DS18B20:
      - Digital sensor.  Much easier to use.  Connects with a single data wire.
      - Requires the 'onewire' and 'ds18x20' libraries (built into ESP32 firmware).
    """

    def __init__(self, sensor_type, pin_number):
        self.sensor_type = sensor_type

        if sensor_type == "NTC":
            # ADC initialisation differs between boards:
            #   Pico  (rp2)  — no attenuation; read_u16() returns 0–65535
            #   ESP32        — needs ATTN_11DB for 3.3 V range; read() returns 0–4095
            if _IS_PICO:
                self.adc = ADC(Pin(pin_number))
                self._adc_read = lambda: self.adc.read_u16()
            else:
                try:
                    self.adc = ADC(Pin(pin_number), atten=ADC.ATTN_11DB)
                except TypeError:                       # firmware < 1.17
                    self.adc = ADC(Pin(pin_number))
                    self.adc.atten(ADC.ATTN_11DB)
                self._adc_read = lambda: self.adc.read()

        elif sensor_type == "DS18B20":
            import onewire
            import ds18x20
            self.ds_pin    = Pin(pin_number)
            self.ow        = onewire.OneWire(self.ds_pin)
            self.ds_sensor = ds18x20.DS18X20(self.ow)
            self.devices   = self.ds_sensor.scan()  # Find sensor on the wire
            if not self.devices:
                raise Exception("No DS18B20 sensor found! Check wiring.")
            print(f"DS18B20 found: {self.devices}")

        else:
            raise Exception(f"Unknown sensor type: {sensor_type}")

    def read_celsius(self):
        """
        Returns temperature in °C.
        Returns None if reading fails (so we can handle errors safely).
        """
        try:
            if self.sensor_type == "NTC":
                return self._read_ntc()
            else:
                return self._read_ds18b20()
        except Exception as e:
            print(f"[SENSOR ERROR] {e}")
            return None

    def _read_ntc(self):
        """
        NTC thermistor reading using the Steinhart-Hart equation.

        Step 1: Read ADC voltage
        Step 2: Calculate NTC resistance from voltage divider
        Step 3: Convert resistance to temperature (Steinhart-Hart simplified)
        """
        raw = self._adc_read()          # 0–4095 on ESP32 | 0–65535 on Pico

        # Avoid division by zero at extremes
        if raw <= 10 or raw >= _ADC_MAX - 10:
            return None

        # Voltage at the ADC pin (volts)
        voltage = (raw / _ADC_MAX) * ADC_VOLTAGE

        # NTC resistance from voltage divider formula:
        # V_out = V_in * R_ntc / (R_pullup + R_ntc)
        # Rearranged: R_ntc = R_pullup * V_out / (V_in - V_out)
        r_ntc = NTC_R_PULLUP * voltage / (ADC_VOLTAGE - voltage)

        # Steinhart-Hart (simplified B-parameter equation):
        # 1/T = 1/T0 + (1/B) * ln(R/R0)
        # T0 = 25°C in Kelvin = 298.15 K
        t_kelvin = 1.0 / (1.0 / 298.15 + math.log(r_ntc / NTC_R_AT_25C) / NTC_B_VALUE)
        return t_kelvin - 273.15   # Convert Kelvin to Celsius

    def _read_ds18b20(self):
        """DS18B20 digital sensor — much simpler than NTC."""
        self.ds_sensor.convert_temp()   # Start conversion (takes ~750 ms)
        time.sleep_ms(800)              # Wait for conversion to finish
        temp = self.ds_sensor.read_temp(self.devices[0])
        return temp


# ============================================================
#  SECTION 4 — PID CONTROLLER
# ============================================================

class PIDController:
    """
    PID = Proportional + Integral + Derivative controller.

    Imagine driving a car and wanting to reach 60 km/h:
      P = "I'm 20 km/h too slow, press gas harder"   (reacts to current error)
      I = "I've been slow for a while, press even more" (corrects long-term drift)
      D = "I'm catching up fast, ease off a bit"      (prevents overshooting)

    This class does the same thing for temperature!
    """

    def __init__(self, kp, ki, kd, setpoint):
        self.kp        = kp          # Proportional gain
        self.ki        = ki          # Integral gain
        self.kd        = kd          # Derivative gain
        self.setpoint  = setpoint    # Target value (desired temperature)

        # Internal state — these are updated every loop
        self._integral  = 0.0        # Accumulated error over time
        self._prev_error = 0.0       # Error from the previous loop (for derivative)
        self._last_time  = None      # When we last computed (for calculating dt)

        # Store the most recent P, I, D values for logging
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def compute(self, measured_value):
        """
        Call this every loop with the current sensor reading.
        Returns a control output between 0 and 100 (percent).
        """
        # Calculate dt (time since last call, in seconds)
        now = time.ticks_ms()
        if self._last_time is None:
            dt = LOOP_INTERVAL_MS / 1000.0   # First run: use default interval
        else:
            dt = time.ticks_diff(now, self._last_time) / 1000.0
        self._last_time = now

        # Avoid division problems if dt is too small
        if dt < 0.001:
            dt = 0.001

        # --- Calculate error ---
        # error > 0  means we're below target (need more heat)
        # error < 0  means we're above target (need less heat)
        error = self.setpoint - measured_value

        # --- P term: proportional to current error ---
        p_term = self.kp * error

        # --- I term: accumulated error over time ---
        self._integral += error * dt

        # Anti-windup: stop integral from growing out of control
        # Without this, the integral could keep growing and cause overshoot
        if self.ki > 0.0001:
            max_integral = 100.0 / self.ki   # Limit so I term stays within 0-100
            self._integral = max(-max_integral, min(max_integral, self._integral))

        i_term = self.ki * self._integral

        # --- D term: rate of change of error ---
        # (how fast is the error changing?)
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Save individual terms for display/logging
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term

        # Add P + I + D, then clamp to 0–100%
        output = p_term + i_term + d_term
        output = max(0.0, min(100.0, output))
        return output

    def reset(self):
        """Reset internal state (call this after a safety shutdown)."""
        self._integral  = 0.0
        self._prev_error = 0.0
        self._last_time  = None


# ============================================================
#  SECTION 5 — OUTPUT DRIVER (PWM / Relay)
# ============================================================

class OutputDriver:
    """
    Controls the relay, SSR (solid state relay), or MOSFET.

    RELAY_MODE = True:
      - Output is either fully ON or fully OFF
      - Simple mechanical relay (can't do PWM, it would wear out fast)
      - Turns on if PID output > 50%, turns off below 50%

    RELAY_MODE = False:
      - Output is a PWM signal (0–100% duty cycle)
      - Use with SSR or MOSFET — they can switch fast
      - Allows smooth power control
    """

    def __init__(self, pin_number, relay_mode,
                 freq_hz, duty_min_pct, duty_max_pct):
        self.relay_mode   = relay_mode
        self.duty_min     = duty_min_pct    # % minimum
        self.duty_max     = duty_max_pct    # % maximum
        self.current_duty = 0.0             # Track current output

        if relay_mode:
            # Digital output — just HIGH or LOW
            self.pin = Pin(pin_number, Pin.OUT)
            self.pin.value(0)               # Start with output OFF
        else:
            # PWM output — frequency in Hz, duty is 0–1023 on ESP32
            self.pwm = PWM(Pin(pin_number), freq=freq_hz)
            self.pwm.duty(0)                # Start at 0% duty

        print(f"[OUTPUT] Mode: {'Relay' if relay_mode else 'PWM'} | "
              f"Freq: {freq_hz} Hz | Duty: {duty_min_pct}–{duty_max_pct}%")

    def set_output(self, pid_output_pct):
        """
        Apply PID output (0–100%) to the physical output.
        Respects the min/max duty limits you set.
        """
        # Apply min/max duty limits
        # If PID says 5% but minimum is 10%, use 10%
        # If PID says 95% but maximum is 80%, use 80%
        if pid_output_pct <= 0.5:
            duty = 0.0    # Treat very small output as fully off
        else:
            duty = max(self.duty_min, min(self.duty_max, pid_output_pct))

        self.current_duty = duty

        if self.relay_mode:
            # Relay: ON if output > 50%, OFF otherwise
            self.pin.value(1 if duty > 50.0 else 0)
        else:
            # PWM: convert % to 0–1023 (ESP32 PWM range)
            pwm_value = int(duty / 100.0 * 1023)
            self.pwm.duty(pwm_value)

    def turn_off(self):
        """Emergency off — cut output completely."""
        self.current_duty = 0.0
        if self.relay_mode:
            self.pin.value(0)
        else:
            self.pwm.duty(0)


# ============================================================
#  SECTION 6 — SAFETY SYSTEM
# ============================================================

class SafetySystem:
    """
    Monitors temperature and blocks output if limits are exceeded.

    Three levels of protection:
      1. WARNING  — Temperature slightly above setpoint. Log and continue.
      2. CUTOFF   — Temperature well above setpoint. Stop output but keep running.
      3. HARD MAX — Absolute maximum reached. Emergency shutdown.

    The system will only allow output when temperature is safe.
    """

    # States
    OK       = "OK"
    WARNING  = "WARNING"
    CUTOFF   = "CUTOFF"
    SHUTDOWN = "SHUTDOWN"

    def __init__(self, setpoint, warn_pct, cutoff_pct, hard_max_c):
        self.setpoint  = setpoint
        self.hard_max  = hard_max_c

        # Calculate actual temperature thresholds from percentages
        # Example: setpoint=60°C, 10% above = 66°C
        margin       = setpoint * (warn_pct   / 100.0)
        self.warn_c  = setpoint + margin
        margin       = setpoint * (cutoff_pct / 100.0)
        self.cutoff_c = setpoint + margin

        self.state         = self.OK
        self.is_blocked    = False           # True = output must be off
        self.shutdown_count = 0             # How many times we hit hard limit

        print(f"[SAFETY] Setpoint: {setpoint}°C | "
              f"Warn: {self.warn_c:.1f}°C | "
              f"Cutoff: {self.cutoff_c:.1f}°C | "
              f"Hard max: {hard_max_c}°C")

    def check(self, temperature):
        """
        Check temperature and update safety state.
        Returns True if output is ALLOWED, False if output must be OFF.
        """
        if temperature is None:
            # Can't read sensor — play it safe and block
            print("[SAFETY] Sensor read failed! Blocking output.")
            self.state      = self.WARNING
            self.is_blocked = True
            return False

        # --- Level 3: Hard maximum ---
        if temperature >= self.hard_max:
            if self.state != self.SHUTDOWN:
                self.shutdown_count += 1
                print(f"[SAFETY] ⚠ EMERGENCY SHUTDOWN! {temperature:.1f}°C "
                      f">= hard max {self.hard_max}°C")
            self.state      = self.SHUTDOWN
            self.is_blocked = True
            return False

        # --- Level 2: Cutoff ---
        if temperature >= self.cutoff_c:
            if self.state not in (self.CUTOFF, self.SHUTDOWN):
                print(f"[SAFETY] ⚠ CUTOFF — {temperature:.1f}°C "
                      f">= {self.cutoff_c:.1f}°C. Output blocked.")
            self.state      = self.CUTOFF
            self.is_blocked = True
            return False

        # --- Level 1: Warning ---
        if temperature >= self.warn_c:
            if self.state == self.OK:
                print(f"[SAFETY] ⚡ WARNING — {temperature:.1f}°C "
                      f">= {self.warn_c:.1f}°C")
            self.state      = self.WARNING
            self.is_blocked = False    # Still allow output but log warning
            return True

        # --- All OK ---
        if self.state in (self.CUTOFF, self.SHUTDOWN):
            # Temperature came back down — allow output again (unless shutdown)
            if self.state != self.SHUTDOWN:
                print(f"[SAFETY] ✓ Temperature safe again ({temperature:.1f}°C). "
                      "Output re-enabled.")
        self.state      = self.OK
        self.is_blocked = False
        return True


# ============================================================
#  SECTION 7 — AUTO-TUNER  (the learning part)
# ============================================================

class AutoTuner:
    """
    A simple learning system that watches PID performance
    and slowly adjusts gains to improve accuracy.

    This is like a teacher watching a student and giving small hints:
      - If the temperature oscillates (goes up/down/up/down):
          → Reduce Kp (less aggressive), increase Kd (more damping)
      - If there's a steady offset below target (never quite reaches it):
          → Increase Ki (push harder over time)
      - If response is too slow (takes too long to reach target):
          → Gently increase Kp

    Changes are small and gradual to avoid making things worse.

    This is a simplified form of machine learning called
    "performance-based gain adaptation" — the system adapts
    based on measured real-world performance metrics.
    """

    def __init__(self, pid, tune_every_s, step_size):
        self.pid         = pid           # Reference to the PID controller
        self.tune_every  = tune_every_s  # Seconds between tune checks
        self.step        = step_size     # How much to nudge gains (e.g. 0.05 = 5%)

        # Performance tracking buffers
        # We collect errors over the tune interval, then analyse them
        self.error_history   = []    # Recent errors (positive or negative)
        self.last_tune_time  = time.ticks_ms()

        # Track number of sign changes in error (indicates oscillation)
        self.prev_error_sign = 0
        self.sign_changes    = 0

        # Stats for display
        self.tune_count  = 0
        self.last_action = "None"

    def record(self, temperature):
        """Call this every loop to collect data for the tuner."""
        error = self.pid.setpoint - temperature
        self.error_history.append(error)

        # Count oscillations (sign changes in error)
        sign = 1 if error > 0 else (-1 if error < 0 else 0)
        if sign != 0 and sign != self.prev_error_sign and self.prev_error_sign != 0:
            self.sign_changes += 1
        self.prev_error_sign = sign

        # Check if it's time to tune
        elapsed = time.ticks_diff(time.ticks_ms(), self.last_tune_time) / 1000.0
        if elapsed >= self.tune_every:
            self._tune()

    def _tune(self):
        """
        Analyse recent performance and adjust PID gains.
        Called automatically every TUNE_EVERY_SECONDS.
        """
        if len(self.error_history) < 5:
            return   # Not enough data yet

        # --- Calculate performance metrics ---
        n = len(self.error_history)

        # Average error (positive = consistently below target)
        avg_error = sum(self.error_history) / n

        # Average absolute error (ignoring direction — how far off are we?)
        avg_abs_error = sum(abs(e) for e in self.error_history) / n

        # Oscillation rate (sign changes per minute)
        # High rate means the system is hunting/oscillating
        elapsed_min  = (self.tune_every / 60.0)
        oscillations = self.sign_changes / max(elapsed_min, 0.01)

        actions = []

        # --- Decision rules ---

        # Rule 1: Too many oscillations → reduce Kp, increase Kd
        # (system is "hunting" — overshooting in both directions)
        if oscillations > 4:   # More than 4 oscillations per minute
            new_kp = self.pid.kp * (1.0 - self.step)
            new_kd = self.pid.kd * (1.0 + self.step)
            self.pid.kp = max(KP_MIN, min(KP_MAX, new_kp))
            self.pid.kd = max(KD_MIN, min(KD_MAX, new_kd))
            actions.append(f"Oscillation ({oscillations:.1f}/min) → Kp↓ Kd↑")

        # Rule 2: Steady offset (consistently above or below target)
        # → increase Ki to push it toward the target
        elif abs(avg_error) > 1.0:   # Offset bigger than 1°C
            new_ki = self.pid.ki * (1.0 + self.step)
            self.pid.ki = max(KI_MIN, min(KI_MAX, new_ki))
            actions.append(f"Offset ({avg_error:.2f}°C) → Ki↑")

        # Rule 3: Response very slow (large average error, no oscillations)
        # → gently increase Kp
        elif avg_abs_error > 5.0 and oscillations < 1:
            new_kp = self.pid.kp * (1.0 + self.step * 0.5)   # Half step — be careful
            self.pid.kp = max(KP_MIN, min(KP_MAX, new_kp))
            actions.append(f"Slow response (avg err {avg_abs_error:.2f}°C) → Kp↑")

        # Rule 4: Performance looks good — don't change anything
        else:
            actions.append(f"Performance OK (avg err {avg_abs_error:.2f}°C)")

        self.last_action = " | ".join(actions)
        self.tune_count += 1

        print(f"[TUNER #{self.tune_count}] {self.last_action}")
        print(f"  Kp={self.pid.kp:.3f}  Ki={self.pid.ki:.3f}  Kd={self.pid.kd:.3f}")

        # --- Reset buffers for next interval ---
        self.error_history  = []
        self.sign_changes   = 0
        self.last_tune_time = time.ticks_ms()


# ============================================================
#  SECTION 8 — STATUS LED
# ============================================================

class StatusLED:
    """
    Blinks the built-in LED to show system status at a glance.

    Fast blink  = output is active (heating)
    Slow blink  = output off, monitoring
    Solid ON    = WARNING or CUTOFF
    Solid OFF   = SHUTDOWN (emergency)
    """

    def __init__(self, pin_number):
        self.led     = Pin(pin_number, Pin.OUT)
        self.led.value(0)
        self._state  = 0
        self._counter = 0

    def update(self, safety_state, output_active):
        self._counter += 1

        if safety_state == SafetySystem.SHUTDOWN:
            self.led.value(0)                # Solid OFF — emergency

        elif safety_state in (SafetySystem.WARNING, SafetySystem.CUTOFF):
            self.led.value(1)                # Solid ON — attention needed

        elif output_active:
            # Fast blink (every loop cycle)
            self._state = 1 - self._state
            self.led.value(self._state)

        else:
            # Slow blink (every 4 loop cycles)
            if self._counter % 4 == 0:
                self._state = 1 - self._state
            self.led.value(self._state)


# ============================================================
#  SECTION 9 — MAIN PROGRAM
# ============================================================

def main():
    """
    This is where everything starts.
    Sets up all components, then runs the control loop forever.
    """

    print("=" * 50)
    print(" ESP32 PID Controller  —  Starting up")
    print("=" * 50)
    print(f" Target temperature : {TARGET_TEMP} °C")
    print(f" Sensor type        : {SENSOR_TYPE}")
    print(f" Output mode        : {'Relay' if RELAY_MODE else 'PWM'}")
    print(f" Auto-Tuner         : {'ON' if AUTOTUNER_ENABLED else 'OFF'}")
    print("=" * 50)

    # --- Create all the objects ---

    sensor = TemperatureSensor(
        sensor_type=SENSOR_TYPE,
        pin_number=PIN_SENSOR,
    )

    pid = PIDController(
        kp=KP_START,
        ki=KI_START,
        kd=KD_START,
        setpoint=TARGET_TEMP,
    )

    output = OutputDriver(
        pin_number=PIN_OUTPUT,
        relay_mode=RELAY_MODE,
        freq_hz=PWM_FREQUENCY_HZ,
        duty_min_pct=PWM_DUTY_MIN_PCT,
        duty_max_pct=PWM_DUTY_MAX_PCT,
    )

    safety = SafetySystem(
        setpoint=TARGET_TEMP,
        warn_pct=SAFETY_WARN_PCT,
        cutoff_pct=SAFETY_CUTOFF_PCT,
        hard_max_c=SAFETY_HARD_MAX_C,
    )

    tuner = AutoTuner(
        pid=pid,
        tune_every_s=TUNE_EVERY_SECONDS,
        step_size=TUNE_STEP_SIZE,
    )

    led = StatusLED(pin_number=PIN_LED)

    # --- Display initialisation ---
    display = None
    if _display_class is not None:
        try:
            if DISPLAY_TYPE == "OLED":
                display = _display_class(
                    sda_pin  = OLED_SDA_PIN,
                    scl_pin  = OLED_SCL_PIN,
                    i2c_addr = OLED_I2C_ADDR,
                )
            elif DISPLAY_TYPE == "IPS":
                display = _display_class(
                    sck_pin  = IPS_SCK_PIN,
                    mosi_pin = IPS_MOSI_PIN,
                    cs_pin   = IPS_CS_PIN,
                    dc_pin   = IPS_DC_PIN,
                    rst_pin  = IPS_RST_PIN,
                    bl_pin   = IPS_BL_PIN,
                    width    = IPS_WIDTH,
                    height   = IPS_HEIGHT,
                    x_offset = IPS_X_OFFSET,
                    y_offset = IPS_Y_OFFSET,
                )
            print(f"[DISPLAY] {DISPLAY_TYPE} initialised OK")
        except Exception as e:
            # Display failure must never crash the PID loop
            print(f"[DISPLAY] Init failed: {e}  — continuing without display")
            display = None

    # --- Main loop variables ---
    loop_count = 0
    last_loop  = time.ticks_ms()

    # --- Serial output header ---
    if SERIAL_PRINT:
        print(f"[SERIAL] Output enabled | Baud: {SERIAL_BAUD} | Format: {SERIAL_FORMAT}")
        if SERIAL_FORMAT == "CSV":
            _serial_write("timestamp_ms,temperature_C,setpoint_C,error,output_pct,P,I,D,safety")

    print("\nRunning! Press Ctrl+C to stop.\n")

    # --- Control loop ---
    # This runs forever, reading sensor and updating output
    while True:

        # ── Wait for next loop interval ──────────────────────
        # This makes the loop run exactly every LOOP_INTERVAL_MS
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, last_loop)
        if elapsed < LOOP_INTERVAL_MS:
            time.sleep_ms(LOOP_INTERVAL_MS - elapsed)
        last_loop = time.ticks_ms()

        loop_count += 1

        # ── Step 1: Read temperature ─────────────────────────
        temperature = sensor.read_celsius()

        # ── Step 2: Check safety ─────────────────────────────
        output_allowed = safety.check(temperature)

        if temperature is None:
            output.turn_off()
            led.update(safety.state, False)
            continue   # Skip rest of loop if sensor failed

        # ── Step 3: Run PID ──────────────────────────────────
        pid_output = pid.compute(temperature)

        # ── Step 4: Apply to output ──────────────────────────
        if output_allowed:
            output.set_output(pid_output)
            is_active = output.current_duty > 0
        else:
            output.turn_off()
            is_active = False
            # Reset PID integral to prevent windup during shutdown
            if safety.state in (SafetySystem.CUTOFF, SafetySystem.SHUTDOWN):
                pid.reset()

        # ── Step 5: Update Auto-Tuner ────────────────────────
        if AUTOTUNER_ENABLED:
            tuner.record(temperature)

        # ── Step 6: Update LED ───────────────────────────────
        led.update(safety.state, is_active)

        # ── Step 7: Update display ───────────────────────────
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
                    tune_count   = tuner.tune_count  if AUTOTUNER_ENABLED else 0,
                    last_action  = tuner.last_action if AUTOTUNER_ENABLED else "disabled",
                    loop_count   = loop_count,
                )
            except Exception as e:
                print(f"[DISPLAY] update error: {e}")

        # ── Step 8: Print status log ─────────────────────────
        if loop_count % LOG_EVERY_LOOPS == 0:
            error = TARGET_TEMP - temperature
            print(
                f"T={temperature:6.2f}°C  "
                f"SP={TARGET_TEMP:.1f}°C  "
                f"Err={error:+6.2f}  "
                f"Out={output.current_duty:5.1f}%  "
                f"P={pid.last_p:+6.2f}  "
                f"I={pid.last_i:+6.2f}  "
                f"D={pid.last_d:+6.2f}  "
                f"Safety={safety.state}"
            )

        # ── Step 9: Serial data output ───────────────────────
        if SERIAL_PRINT and loop_count % SERIAL_EVERY_LOOPS == 0:
            ts  = time.ticks_ms()
            err = TARGET_TEMP - temperature
            if SERIAL_FORMAT == "CSV":
                _serial_write(
                    f"{ts},{temperature:.2f},{TARGET_TEMP:.1f},{err:.2f},"
                    f"{output.current_duty:.1f},{pid.last_p:.2f},"
                    f"{pid.last_i:.2f},{pid.last_d:.2f},{safety.state}"
                )
            else:  # HUMAN
                _serial_write(
                    f"[DATA] T={temperature:.2f}C  SP={TARGET_TEMP:.1f}C  "
                    f"Err={err:+.2f}  Out={output.current_duty:.1f}%  "
                    f"P={pid.last_p:+.2f}  I={pid.last_i:+.2f}  "
                    f"D={pid.last_d:+.2f}  Safety={safety.state}"
                )


# ============================================================
#  START
# ============================================================

# This check runs the main function when you upload this file
# (Safe to have in MicroPython — it just starts immediately)
try:
    main()
except KeyboardInterrupt:
    # User pressed Ctrl+C — clean shutdown
    print("\nStopped by user. Turning output off.")
    # Turn off output pin safely on exit
    Pin(PIN_OUTPUT, Pin.OUT).value(0)
    print("Output OFF. Goodbye.")
