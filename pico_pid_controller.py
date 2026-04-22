# ============================================================
#  Raspberry Pi Pico PID Controller  —  MicroPython
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
#  WIRING  (Pico GP pin numbers)
# ============================================================
#
#  NTC Thermistor (temperature sensor):
#    3.3V ──── NTC ──── GP26 (ADC0) ──── 10kΩ ──── GND
#                            ↑ read voltage here
#
#  DS18B20 (alternative digital sensor):
#    3.3V ──── DS18B20 VCC
#    GND  ──── DS18B20 GND
#    GP26  ─── DS18B20 DATA ──── 4.7kΩ ──── 3.3V   (pull-up!)
#
#  Relay / SSR / MOSFET:
#    GP16 ──── relay IN  (controls your heater)
#
#  Status LED:
#    GP25 = built-in LED on Pico  (use Pin("LED") on Pico W)
#
#  OLED display (SSD1306 I2C):
#    GP14 ──── SDA
#    GP15 ──── SCL
#
# ============================================================


# ============================================================
#  SECTION 1 — SETTINGS
#  This is the only section you need to edit!
# ============================================================

# ── What temperature do you want to reach? ──────────────────
TARGET_TEMP = 60.0          # Target in °C

# ── PID Gains (starting values — Auto-Tuner will improve them)
KP_START = 5.0
KI_START = 0.05
KD_START = 1.0

# ── PWM output settings ─────────────────────────────────────
PWM_FREQUENCY_HZ = 1        # Hz.  Software time-proportioning — no hardware limit.
                             # 1 Hz = 1 s cycle (good for thermal systems).
                             # 0.5 = 2 s cycle,  2 = 500 ms cycle, etc.
PWM_DUTY_MIN_PCT = 0        # % — minimum duty cycle when output is active
PWM_DUTY_MAX_PCT = 100      # % — maximum duty cycle (hard ceiling for safety)

# ── Control mode ─────────────────────────────────────────────
RELAY_MODE = False           # True = digital relay (ON/OFF), False = PWM

# ── Sensor type ──────────────────────────────────────────────
SENSOR_TYPE = "NTC"         # "NTC" = thermistor | "DS18B20" = digital sensor

# ── GPIO pin numbers (Pico GP numbers) ──────────────────────
PIN_SENSOR  = 26            # ADC0 — NTC thermistor or DS18B20 data
PIN_OUTPUT  = 16            # Relay / SSR / MOSFET control
PIN_LED     = 25            # Built-in LED (use "LED" string on Pico W)

# ── NTC thermistor calibration ───────────────────────────────
NTC_R_PULLUP  = 10_000      # Ω  — fixed resistor to GND
NTC_R_AT_25C  = 10_000      # Ω  — NTC resistance at 25 °C
NTC_B_VALUE   = 3950        # B-coefficient from NTC datasheet
ADC_MAX       = 65535       # Pico ADC is 16-bit (0–65535)
ADC_VOLTAGE   = 3.3         # Reference voltage

# ── Safety limits ────────────────────────────────────────────
SAFETY_HARD_MAX_C   = 90.0  # °C  — ABSOLUTE LIMIT. Output shuts off completely.
SAFETY_WARN_PCT     = 10.0  # %   — Warning zone above setpoint
SAFETY_CUTOFF_PCT   = 20.0  # %   — Emergency cutoff above setpoint

# ── Auto-Tuner settings ──────────────────────────────────────
AUTOTUNER_ENABLED   = True
TUNE_EVERY_SECONDS  = 30
TUNE_STEP_SIZE      = 0.05
KP_MIN, KP_MAX = 0.5, 50.0
KI_MIN, KI_MAX = 0.0,  5.0
KD_MIN, KD_MAX = 0.0, 10.0

# ── Loop timing ──────────────────────────────────────────────
LOOP_INTERVAL_MS  = 500
LOG_EVERY_LOOPS   = 10

# ── Display settings ─────────────────────────────────────────
# Choose ONE of:  "OLED"  |  "IPS"  |  "NONE"
DISPLAY_TYPE = "NONE"

# OLED — SSD1306 128×64 via I2C on GP14 (SDA) / GP15 (SCL)
OLED_SDA_PIN   = 14
OLED_SCL_PIN   = 15
OLED_I2C_ADDR  = 0x3C        # try 0x3D if display not found

# IPS — ST7789 color display via SPI
IPS_SCK_PIN    = 18
IPS_MOSI_PIN   = 19
IPS_CS_PIN     = 17
IPS_DC_PIN     = 20
IPS_RST_PIN    = 21
IPS_BL_PIN     = -1
IPS_WIDTH      = 240
IPS_HEIGHT     = 240
IPS_X_OFFSET   = 0
IPS_Y_OFFSET   = 0

DISPLAY_UPDATE_EVERY = 2

# ── Serial data output ────────────────────────────────────────
SERIAL_PRINT        = True
SERIAL_BAUD         = 115200
SERIAL_FORMAT       = "CSV"
SERIAL_EVERY_LOOPS  = 1


# ============================================================
#  SECTION 2 — IMPORTS
# ============================================================

from machine import Pin, ADC
import time
import math
import sys as _sys

_IS_PICO  = _sys.platform == "rp2"
_IS_ESP32 = _sys.platform == "esp32"
_ADC_MAX  = 65535   # Pico 16-bit ADC
print(f"[BOARD] Platform: {_sys.platform} | ADC max: {_ADC_MAX}")

if SERIAL_PRINT:
    import sys as _sys

def _serial_write(line):
    if SERIAL_PRINT:
        _sys.stdout.write(line + "\r\n")

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
    def __init__(self, sensor_type, pin_number):
        self.sensor_type = sensor_type

        if sensor_type == "NTC":
            # Pico: no attenuation needed, use read_u16() for 0–65535
            self.adc = ADC(Pin(pin_number))
            self._adc_read = lambda: self.adc.read_u16()

        elif sensor_type == "DS18B20":
            import onewire
            import ds18x20
            self.ds_pin    = Pin(pin_number)
            self.ow        = onewire.OneWire(self.ds_pin)
            self.ds_sensor = ds18x20.DS18X20(self.ow)
            self.devices   = self.ds_sensor.scan()
            if not self.devices:
                raise Exception("No DS18B20 sensor found! Check wiring.")
            print(f"DS18B20 found: {self.devices}")

        else:
            raise Exception(f"Unknown sensor type: {sensor_type}")

    def read_celsius(self):
        try:
            if self.sensor_type == "NTC":
                return self._read_ntc()
            else:
                return self._read_ds18b20()
        except Exception as e:
            print(f"[SENSOR ERROR] {e}")
            return None

    def _read_ntc(self):
        raw = self._adc_read()   # 0–65535

        if raw <= 10 or raw >= _ADC_MAX - 10:
            return None

        voltage = (raw / _ADC_MAX) * ADC_VOLTAGE
        r_ntc   = NTC_R_PULLUP * (ADC_VOLTAGE - voltage) / voltage
        t_kelvin = 1.0 / (1.0 / 298.15 + math.log(r_ntc / NTC_R_AT_25C) / NTC_B_VALUE)
        return t_kelvin - 273.15

    def _read_ds18b20(self):
        self.ds_sensor.convert_temp()
        time.sleep_ms(800)
        return self.ds_sensor.read_temp(self.devices[0])


# ============================================================
#  SECTION 4 — PID CONTROLLER
# ============================================================

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp        = kp
        self.ki        = ki
        self.kd        = kd
        self.setpoint  = setpoint
        self._integral  = 0.0
        self._prev_error = 0.0
        self._last_time  = None
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def compute(self, measured_value):
        now = time.ticks_ms()
        if self._last_time is None:
            dt = LOOP_INTERVAL_MS / 1000.0
        else:
            dt = time.ticks_diff(now, self._last_time) / 1000.0
        self._last_time = now

        if dt < 0.001:
            dt = 0.001

        error  = self.setpoint - measured_value
        p_term = self.kp * error

        self._integral += error * dt
        if self.ki > 0.0001:
            max_integral = 100.0 / self.ki
            self._integral = max(-max_integral, min(max_integral, self._integral))
        i_term = self.ki * self._integral

        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term

        output = p_term + i_term + d_term
        return max(0.0, min(100.0, output))

    def reset(self):
        self._integral  = 0.0
        self._prev_error = 0.0
        self._last_time  = None


# ============================================================
#  SECTION 5 — OUTPUT DRIVER (PWM / Relay)
# ============================================================

class OutputDriver:
    def __init__(self, pin_number, relay_mode,
                 freq_hz, duty_min_pct, duty_max_pct):
        self.relay_mode   = relay_mode
        self.duty_min     = duty_min_pct
        self.duty_max     = duty_max_pct
        self.current_duty = 0.0

        # Single digital pin for both relay and soft-PWM modes
        self.pin = Pin(pin_number, Pin.OUT)
        self.pin.value(0)

        if not relay_mode:
            # Software time-proportioning: no hardware PWM, no frequency floor.
            # Each cycle = period_ms long; pin is ON for the first (duty%) of it.
            self.period_ms    = int(1000 / freq_hz)
            self._cycle_start = time.ticks_ms()

        print(f"[OUTPUT] Mode: {'Relay' if relay_mode else 'Soft-PWM'} | "
              f"Freq: {freq_hz} Hz | Duty: {duty_min_pct}–{duty_max_pct}%")

    def set_output(self, pid_output_pct):
        if pid_output_pct <= 0.5:
            duty = 0.0
        else:
            duty = max(self.duty_min, min(self.duty_max, pid_output_pct))

        self.current_duty = duty

        if self.relay_mode:
            self.pin.value(1 if duty > 50.0 else 0)
        # Soft-PWM: tick() handles the pin each loop iteration

    def tick(self):
        """Drive the on/off cycle for soft-PWM. Call every main loop iteration."""
        if self.relay_mode:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self._cycle_start)
        if elapsed >= self.period_ms:
            self._cycle_start = time.ticks_ms()
            elapsed = 0
        on_ms = int(self.current_duty / 100.0 * self.period_ms)
        self.pin.value(1 if (on_ms > 0 and elapsed < on_ms) else 0)

    def turn_off(self):
        self.current_duty = 0.0
        self.pin.value(0)


# ============================================================
#  SECTION 6 — SAFETY SYSTEM
# ============================================================

class SafetySystem:
    OK       = "OK"
    WARNING  = "WARNING"
    CUTOFF   = "CUTOFF"
    SHUTDOWN = "SHUTDOWN"

    def __init__(self, setpoint, warn_pct, cutoff_pct, hard_max_c):
        self.setpoint  = setpoint
        self.hard_max  = hard_max_c
        margin         = setpoint * (warn_pct   / 100.0)
        self.warn_c    = setpoint + margin
        margin         = setpoint * (cutoff_pct / 100.0)
        self.cutoff_c  = setpoint + margin
        self.state          = self.OK
        self.is_blocked     = False
        self.shutdown_count = 0

        print(f"[SAFETY] Setpoint: {setpoint}°C | "
              f"Warn: {self.warn_c:.1f}°C | "
              f"Cutoff: {self.cutoff_c:.1f}°C | "
              f"Hard max: {hard_max_c}°C")

    def check(self, temperature):
        if temperature is None:
            print("[SAFETY] Sensor read failed! Blocking output.")
            self.state      = self.WARNING
            self.is_blocked = True
            return False

        if temperature >= self.hard_max:
            if self.state != self.SHUTDOWN:
                self.shutdown_count += 1
                print(f"[SAFETY] EMERGENCY SHUTDOWN! {temperature:.1f}°C "
                      f">= hard max {self.hard_max}°C")
            self.state      = self.SHUTDOWN
            self.is_blocked = True
            return False

        if temperature >= self.cutoff_c:
            if self.state not in (self.CUTOFF, self.SHUTDOWN):
                print(f"[SAFETY] CUTOFF — {temperature:.1f}°C "
                      f">= {self.cutoff_c:.1f}°C. Output blocked.")
            self.state      = self.CUTOFF
            self.is_blocked = True
            return False

        if temperature >= self.warn_c:
            if self.state == self.OK:
                print(f"[SAFETY] WARNING — {temperature:.1f}°C "
                      f">= {self.warn_c:.1f}°C")
            self.state      = self.WARNING
            self.is_blocked = False
            return True

        if self.state in (self.CUTOFF, self.SHUTDOWN):
            if self.state != self.SHUTDOWN:
                print(f"[SAFETY] Temperature safe again ({temperature:.1f}°C). "
                      "Output re-enabled.")
        self.state      = self.OK
        self.is_blocked = False
        return True


# ============================================================
#  SECTION 7 — AUTO-TUNER
# ============================================================

class AutoTuner:
    def __init__(self, pid, tune_every_s, step_size):
        self.pid         = pid
        self.tune_every  = tune_every_s
        self.step        = step_size
        self.error_history   = []
        self.last_tune_time  = time.ticks_ms()
        self.prev_error_sign = 0
        self.sign_changes    = 0
        self.tune_count  = 0
        self.last_action = "None"

    def record(self, temperature):
        error = self.pid.setpoint - temperature
        self.error_history.append(error)

        sign = 1 if error > 0 else (-1 if error < 0 else 0)
        if sign != 0 and sign != self.prev_error_sign and self.prev_error_sign != 0:
            self.sign_changes += 1
        self.prev_error_sign = sign

        elapsed = time.ticks_diff(time.ticks_ms(), self.last_tune_time) / 1000.0
        if elapsed >= self.tune_every:
            self._tune()

    def _tune(self):
        if len(self.error_history) < 5:
            return

        n            = len(self.error_history)
        avg_error    = sum(self.error_history) / n
        avg_abs_error = sum(abs(e) for e in self.error_history) / n
        elapsed_min  = (self.tune_every / 60.0)
        oscillations = self.sign_changes / max(elapsed_min, 0.01)
        actions = []

        if oscillations > 4:
            self.pid.kp = max(KP_MIN, min(KP_MAX, self.pid.kp * (1.0 - self.step)))
            self.pid.kd = max(KD_MIN, min(KD_MAX, self.pid.kd * (1.0 + self.step)))
            actions.append(f"Oscillation ({oscillations:.1f}/min) -> Kp- Kd+")
        elif abs(avg_error) > 1.0:
            self.pid.ki = max(KI_MIN, min(KI_MAX, self.pid.ki * (1.0 + self.step)))
            actions.append(f"Offset ({avg_error:.2f}°C) -> Ki+")
        elif avg_abs_error > 5.0 and oscillations < 1:
            self.pid.kp = max(KP_MIN, min(KP_MAX, self.pid.kp * (1.0 + self.step * 0.5)))
            actions.append(f"Slow response (avg err {avg_abs_error:.2f}°C) -> Kp+")
        else:
            actions.append(f"Performance OK (avg err {avg_abs_error:.2f}°C)")

        self.last_action = " | ".join(actions)
        self.tune_count += 1
        print(f"[TUNER #{self.tune_count}] {self.last_action}")
        print(f"  Kp={self.pid.kp:.3f}  Ki={self.pid.ki:.3f}  Kd={self.pid.kd:.3f}")

        self.error_history  = []
        self.sign_changes   = 0
        self.last_tune_time = time.ticks_ms()


# ============================================================
#  SECTION 8 — STATUS LED
# ============================================================

class StatusLED:
    def __init__(self, pin_number):
        self.led      = Pin(pin_number, Pin.OUT)
        self.led.value(0)
        self._state   = 0
        self._counter = 0

    def update(self, safety_state, output_active):
        self._counter += 1

        if safety_state == SafetySystem.SHUTDOWN:
            self.led.value(0)
        elif safety_state in (SafetySystem.WARNING, SafetySystem.CUTOFF):
            self.led.value(1)
        elif output_active:
            self._state = 1 - self._state
            self.led.value(self._state)
        else:
            if self._counter % 4 == 0:
                self._state = 1 - self._state
            self.led.value(self._state)


# ============================================================
#  SECTION 9 — MAIN PROGRAM
# ============================================================

def main():
    print("=" * 50)
    print(" Pico PID Controller  —  Starting up")
    print("=" * 50)
    print(f" Target temperature : {TARGET_TEMP} °C")
    print(f" Sensor type        : {SENSOR_TYPE}")
    print(f" Output mode        : {'Relay' if RELAY_MODE else 'PWM'}")
    print(f" Auto-Tuner         : {'ON' if AUTOTUNER_ENABLED else 'OFF'}")
    print("=" * 50)

    sensor = TemperatureSensor(sensor_type=SENSOR_TYPE, pin_number=PIN_SENSOR)

    pid = PIDController(kp=KP_START, ki=KI_START, kd=KD_START, setpoint=TARGET_TEMP)

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

    tuner = AutoTuner(pid=pid, tune_every_s=TUNE_EVERY_SECONDS, step_size=TUNE_STEP_SIZE)

    led = StatusLED(pin_number=PIN_LED)

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
            print(f"[DISPLAY] Init failed: {e}  — continuing without display")
            display = None

    loop_count = 0
    last_loop  = time.ticks_ms()

    if SERIAL_PRINT:
        print(f"[SERIAL] Output enabled | Baud: {SERIAL_BAUD} | Format: {SERIAL_FORMAT}")
        if SERIAL_FORMAT == "CSV":
            _serial_write("timestamp_ms,temperature_C,setpoint_C,error,output_pct,P,I,D,safety")

    print("\nRunning! Press Ctrl+C to stop.\n")

    while True:
        now     = time.ticks_ms()
        elapsed = time.ticks_diff(now, last_loop)
        if elapsed < LOOP_INTERVAL_MS:
            time.sleep_ms(LOOP_INTERVAL_MS - elapsed)
        last_loop = time.ticks_ms()

        loop_count += 1

        temperature    = sensor.read_celsius()
        output_allowed = safety.check(temperature)

        if temperature is None:
            output.turn_off()
            led.update(safety.state, False)
            continue

        pid_output = pid.compute(temperature)

        if output_allowed:
            output.set_output(pid_output)
            is_active = output.current_duty > 0
        else:
            output.turn_off()
            is_active = False
            if safety.state in (SafetySystem.CUTOFF, SafetySystem.SHUTDOWN):
                pid.reset()

        if AUTOTUNER_ENABLED:
            tuner.record(temperature)

        output.tick()

        led.update(safety.state, is_active)

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

        if SERIAL_PRINT and loop_count % SERIAL_EVERY_LOOPS == 0:
            ts  = time.ticks_ms()
            err = TARGET_TEMP - temperature
            if SERIAL_FORMAT == "CSV":
                _serial_write(
                    f"{ts},{temperature:.2f},{TARGET_TEMP:.1f},{err:.2f},"
                    f"{output.current_duty:.1f},{pid.last_p:.2f},"
                    f"{pid.last_i:.2f},{pid.last_d:.2f},{safety.state}"
                )
            else:
                _serial_write(
                    f"[DATA] T={temperature:.2f}C  SP={TARGET_TEMP:.1f}C  "
                    f"Err={err:+.2f}  Out={output.current_duty:.1f}%  "
                    f"P={pid.last_p:+.2f}  I={pid.last_i:+.2f}  "
                    f"D={pid.last_d:+.2f}  Safety={safety.state}"
                )


# ============================================================
#  START
# ============================================================

try:
    main()
except KeyboardInterrupt:
    print("\nStopped by user. Turning output off.")
    Pin(PIN_OUTPUT, Pin.OUT).value(0)
    print("Output OFF. Goodbye.")
