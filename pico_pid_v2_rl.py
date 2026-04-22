# ============================================================
#  Raspberry Pi Pico  —  Reinforcement Learning Heater Controller
#  MicroPython  |  v2
# ============================================================
#
#  Instead of a fixed PID formula, a Q-learning agent watches
#  the temperature and learns — by trial and error — which
#  heater power level works best in every situation.
#
#  HOW IT WORKS (plain English):
#    The agent sees two things:
#      • How far the temperature is from the target  (error)
#      • Whether the temperature is rising or falling (rate)
#    From those it picks a heater power level (0 / 25 / 50 / 75 / 100 %).
#    After each step it gets a reward:
#      +1.0  if temperature is within 1 °C of target   ← good
#       0.0  if within 3 °C                            ← ok
#      −big  if far away or safety triggered            ← bad
#    It updates a small table (Q-table) so next time it is in
#    the same situation it will tend to make a better choice.
#    The Q-table is saved to flash — so it keeps what it learned
#    even after a power cut.
#
#  WIRING  (same as v1)
#    3.3V ──── NTC ──── GP26 (ADC0) ──── 100kΩ ──── GND
#    GP16 ──── relay / SSR / MOSFET
#    GP25 ──── built-in LED
#    GP14 ──── OLED SDA  |  GP15 ──── OLED SCL
#
# ============================================================


# ============================================================
#  SECTION 1 — SETTINGS
# ============================================================

TARGET_TEMP = 60.0          # °C — temperature to hold

# ── Hardware pins ────────────────────────────────────────────
PIN_SENSOR  = 26            # ADC0 — NTC thermistor
PIN_OUTPUT  = 16            # Relay / SSR / MOSFET
PIN_LED     = 25            # Built-in LED  (use "LED" on Pico W)

# ── NTC thermistor (100 k) ───────────────────────────────────
NTC_R_SERIES  = 100_000     # Ω  — fixed series resistor (low side)
NTC_R_AT_25C  = 100_000     # Ω  — NTC resistance at 25 °C
NTC_B_VALUE   = 3950        # B-coefficient (check your datasheet)
ADC_VOLTAGE   = 3.3

# ── Heater output ────────────────────────────────────────────
# The agent picks one of these duty-cycle levels each step.
# Add / remove levels to change resolution vs. Q-table size.
OUTPUT_LEVELS = [0, 15, 30, 50, 70, 85, 100]   # % duty

# Soft-PWM cycle length — 1 Hz = 1 s ON+OFF window
PWM_FREQUENCY_HZ = 1

# ── Safety ───────────────────────────────────────────────────
SAFETY_HARD_MAX_C  = 90.0   # °C  — absolute emergency shutoff
SAFETY_CUTOFF_PCT  = 20.0   # %   — cutoff above setpoint  (60 + 20% = 72 °C)
SAFETY_WARN_PCT    = 10.0   # %   — warning above setpoint (60 + 10% = 66 °C)

# ── RL hyper-parameters ──────────────────────────────────────
RL_ALPHA          = 0.15    # Learning rate  (0 = never learn, 1 = forget instantly)
RL_GAMMA          = 0.92    # Discount  (how much future rewards matter, 0–1)
RL_EPSILON_START  = 0.90    # Starting exploration rate  (1.0 = always random)
RL_EPSILON_MIN    = 0.05    # Minimum exploration rate   (always try 5% random)
RL_EPSILON_DECAY  = 0.9995  # Multiply ε by this every step  (slow decay)
RL_SAVE_EVERY     = 200     # Save Q-table to flash every N steps

# File where the Q-table is stored on Pico flash
QTABLE_FILE = "qtable_v2.json"

# ── Display ──────────────────────────────────────────────────
DISPLAY_TYPE = "NONE"       # "OLED" | "NONE"
OLED_SDA_PIN  = 14
OLED_SCL_PIN  = 15
OLED_I2C_ADDR = 0x3C
DISPLAY_UPDATE_EVERY = 4    # refresh every N steps

# ── Loop timing ──────────────────────────────────────────────
LOOP_INTERVAL_MS = 500      # main loop period
LOG_EVERY_LOOPS  = 10       # print to serial every N loops

# ── Serial output ────────────────────────────────────────────
SERIAL_PRINT = True


# ============================================================
#  SECTION 2 — IMPORTS
# ============================================================

from machine import Pin, ADC
import time, math, json, random, sys


def _log(line):
    if SERIAL_PRINT:
        sys.stdout.write(line + "\r\n")


_display_class = None
if DISPLAY_TYPE == "OLED":
    try:
        from display_oled import OLEDDisplay as _display_class
    except ImportError:
        print("[DISPLAY] display_oled.py not found — running without display")


# ============================================================
#  SECTION 3 — TEMPERATURE SENSOR  (NTC thermistor)
# ============================================================

class TemperatureSensor:
    _ADC_MAX = 65535    # Pico 16-bit ADC

    def __init__(self, pin_number):
        self.adc = ADC(Pin(pin_number))

    def read_celsius(self):
        try:
            raw = self.adc.read_u16()
            if raw <= 10 or raw >= self._ADC_MAX - 10:
                return None
            voltage = (raw / self._ADC_MAX) * ADC_VOLTAGE
            # NTC on HIGH side → r_ntc = R_series × (Vin − V) / V
            r_ntc    = NTC_R_SERIES * (ADC_VOLTAGE - voltage) / voltage
            t_kelvin = 1.0 / (1.0 / 298.15 + math.log(r_ntc / NTC_R_AT_25C) / NTC_B_VALUE)
            return t_kelvin - 273.15
        except Exception as e:
            print("[SENSOR]", e)
            return None


# ============================================================
#  SECTION 4 — Q-LEARNING AGENT
# ============================================================
#
#  STATE  = (error_bucket, rate_bucket)
#    error_bucket  — how far temperature is from setpoint
#    rate_bucket   — is temperature rising or falling?
#
#  ACTION = index into OUTPUT_LEVELS  (0 % … 100 %)
#
#  Q-TABLE  maps every (state, action) pair to an expected
#  cumulative reward.  Updated with the Bellman equation:
#
#    Q(s,a) ← Q(s,a) + α · [r + γ · max Q(s',·) − Q(s,a)]
#
#  ε-GREEDY  exploration:
#    With probability ε  → pick a random action  (explore)
#    With probability 1-ε → pick the best known action (exploit)
#    ε decays over time so the agent exploits more as it learns.

class QLearningAgent:

    # Error boundaries in °C  (positive = below setpoint = too cold)
    _ERR_BOUNDS  = [-15, -8, -3, -1, 1, 3, 8, 15]   # 9 buckets
    # Rate boundaries in °C/s  (positive = warming up)
    _RATE_BOUNDS = [-0.8, -0.2, 0.2, 0.8]            # 5 buckets

    _N_ERR    = len(_ERR_BOUNDS)  + 1   # 9
    _N_RATE   = len(_RATE_BOUNDS) + 1   # 5
    _N_ACTION = len(OUTPUT_LEVELS)

    def __init__(self):
        self.alpha         = RL_ALPHA
        self.gamma         = RL_GAMMA
        self.epsilon       = RL_EPSILON_START
        self.epsilon_min   = RL_EPSILON_MIN
        self.epsilon_decay = RL_EPSILON_DECAY

        # Q-table: flat list of floats, all starting at 0
        self._q = [0.0] * (self._N_ERR * self._N_RATE * self._N_ACTION)

        # Previous step memory for Bellman update
        self._prev_s  = None    # (error_i, rate_i)
        self._prev_ai = None    # action index
        self._prev_err = 0.0

        self.steps      = 0
        self.last_duty  = 0     # % — most recent output for display

        self._load()
        print("[RL] State space: {}×{}  Actions: {}  Q-table: {} entries".format(
            self._N_ERR, self._N_RATE, self._N_ACTION, len(self._q)))

    # ── Q-table index helper ──────────────────────────────────
    def _idx(self, ei, ri, ai):
        return (ei * self._N_RATE + ri) * self._N_ACTION + ai

    # ── Discretise a continuous value into a bucket index ─────
    @staticmethod
    def _bucket(value, bounds):
        for i, b in enumerate(bounds):
            if value < b:
                return i
        return len(bounds)

    # ── Best action for a given state ─────────────────────────
    def _best_action(self, ei, ri):
        best_ai, best_q = 0, self._q[self._idx(ei, ri, 0)]
        for ai in range(1, self._N_ACTION):
            q = self._q[self._idx(ei, ri, ai)]
            if q > best_q:
                best_q = q
                best_ai = ai
        return best_ai, best_q

    # ── Reward function ───────────────────────────────────────
    @staticmethod
    def _reward(error, safety_blocked):
        if safety_blocked:
            return -10.0            # hard penalty for triggering safety
        ae = abs(error)
        if ae <= 1.0:   return  1.0
        if ae <= 3.0:   return  0.2
        if ae <= 8.0:   return -ae * 0.1
        return -1.5                 # far from target

    # ── Main entry: observe → learn → act ────────────────────
    def step(self, temperature, setpoint, safety_blocked, dt):
        """
        Call every control loop.
        Returns the duty cycle % to apply to the heater.
        """
        error = setpoint - temperature
        rate  = (error - self._prev_err) / max(dt, 0.001)  # °C/s

        ei = self._bucket(error, self._ERR_BOUNDS)
        ri = self._bucket(rate,  self._RATE_BOUNDS)

        # ── Bellman update for the PREVIOUS step ─────────────
        if self._prev_s is not None:
            pei, pri = self._prev_s
            pai      = self._prev_ai
            reward   = self._reward(error, safety_blocked)
            _, max_q = self._best_action(ei, ri)
            idx      = self._idx(pei, pri, pai)
            self._q[idx] += self.alpha * (
                reward + self.gamma * max_q - self._q[idx]
            )

        # ── Choose action (ε-greedy) ──────────────────────────
        if safety_blocked:
            ai = 0          # forced off during safety event — learn nothing
        elif random.random() < self.epsilon:
            ai = random.randint(0, self._N_ACTION - 1)   # explore
        else:
            ai, _ = self._best_action(ei, ri)            # exploit

        # ── Decay exploration rate ────────────────────────────
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        # ── Save state for next step ──────────────────────────
        self._prev_s   = (ei, ri)
        self._prev_ai  = ai
        self._prev_err = error
        self.steps    += 1
        self.last_duty = OUTPUT_LEVELS[ai]

        if self.steps % RL_SAVE_EVERY == 0:
            self._save()

        return OUTPUT_LEVELS[ai]

    # ── Flash persistence ─────────────────────────────────────
    def _save(self):
        try:
            with open(QTABLE_FILE, "w") as f:
                json.dump(self._q, f)
            print("[RL] Saved  steps={} ε={:.3f}".format(self.steps, self.epsilon))
        except Exception as e:
            print("[RL] Save failed:", e)

    def _load(self):
        try:
            with open(QTABLE_FILE, "r") as f:
                data = json.load(f)
            if len(data) == len(self._q):
                self._q   = data
                # Restore epsilon to minimum so it mostly exploits what it learned
                self.epsilon = self.epsilon_min
                print("[RL] Loaded Q-table ({} entries) — exploiting learned policy".format(len(self._q)))
            else:
                print("[RL] Q-table size mismatch — starting fresh")
        except:
            print("[RL] No saved Q-table — starting fresh (will explore first)")


# ============================================================
#  SECTION 5 — OUTPUT DRIVER  (soft-PWM, any frequency)
# ============================================================

class OutputDriver:
    def __init__(self, pin_number, freq_hz):
        self.pin          = Pin(pin_number, Pin.OUT)
        self.pin.value(0)
        self.period_ms    = int(1000 / freq_hz)
        self._cycle_start = time.ticks_ms()
        self.current_duty = 0.0
        print("[OUTPUT] Soft-PWM  {} Hz  period {} ms".format(freq_hz, self.period_ms))

    def set_duty(self, duty_pct):
        self.current_duty = max(0.0, min(100.0, duty_pct))

    def tick(self):
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

    def __init__(self, setpoint):
        margin         = setpoint * (SAFETY_WARN_PCT   / 100.0)
        self.warn_c    = setpoint + margin
        margin         = setpoint * (SAFETY_CUTOFF_PCT / 100.0)
        self.cutoff_c  = setpoint + margin
        self.hard_max  = SAFETY_HARD_MAX_C
        self.state     = self.OK
        self.blocked   = False
        print("[SAFETY] Warn: {:.1f}°C  Cutoff: {:.1f}°C  Hard max: {:.1f}°C".format(
            self.warn_c, self.cutoff_c, self.hard_max))

    def check(self, temperature):
        if temperature is None:
            self.state   = self.WARNING
            self.blocked = True
            return False

        if temperature >= self.hard_max:
            if self.state != self.SHUTDOWN:
                print("[SAFETY] EMERGENCY SHUTDOWN {:.1f}°C".format(temperature))
            self.state   = self.SHUTDOWN
            self.blocked = True
            return False

        if temperature >= self.cutoff_c:
            if self.state not in (self.CUTOFF, self.SHUTDOWN):
                print("[SAFETY] CUTOFF {:.1f}°C".format(temperature))
            self.state   = self.CUTOFF
            self.blocked = True
            return False

        if temperature >= self.warn_c:
            self.state   = self.WARNING
            self.blocked = False
            return True

        if self.state == self.CUTOFF:
            print("[SAFETY] Resumed {:.1f}°C".format(temperature))
        self.state   = self.OK
        self.blocked = False
        return True


# ============================================================
#  SECTION 7 — STATUS LED
# ============================================================

class StatusLED:
    def __init__(self, pin_number):
        self.led      = Pin(pin_number, Pin.OUT)
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
#  SECTION 8 — MAIN
# ============================================================

def main():
    print("=" * 50)
    print(" Pico RL Heater Controller  v2")
    print("=" * 50)
    print(" Target : {} C".format(TARGET_TEMP))
    print(" Levels : {}".format(OUTPUT_LEVELS))
    print("=" * 50)

    sensor = TemperatureSensor(PIN_SENSOR)
    agent  = QLearningAgent()
    output = OutputDriver(PIN_OUTPUT, PWM_FREQUENCY_HZ)
    safety = SafetySystem(TARGET_TEMP)
    led    = StatusLED(PIN_LED)

    display = None
    if _display_class is not None:
        try:
            display = _display_class(
                sda_pin  = OLED_SDA_PIN,
                scl_pin  = OLED_SCL_PIN,
                i2c_addr = OLED_I2C_ADDR,
            )
            print("[DISPLAY] OLED ready")
        except Exception as e:
            print("[DISPLAY] Init failed:", e)

    if SERIAL_PRINT:
        _log("timestamp_ms,temperature_C,setpoint_C,error,duty_pct,safety,epsilon,steps")

    print("\nRunning — press Ctrl+C to stop.\n")

    loop_count = 0
    last_loop  = time.ticks_ms()
    last_time  = time.ticks_ms()

    while True:
        # ── Timing ───────────────────────────────────────────
        now     = time.ticks_ms()
        elapsed = time.ticks_diff(now, last_loop)
        if elapsed < LOOP_INTERVAL_MS:
            time.sleep_ms(LOOP_INTERVAL_MS - elapsed)
        dt        = time.ticks_diff(time.ticks_ms(), last_time) / 1000.0
        last_time = time.ticks_ms()
        last_loop = last_time
        loop_count += 1

        # ── Read sensor ───────────────────────────────────────
        temperature = sensor.read_celsius()

        # ── Safety check ──────────────────────────────────────
        output_allowed = safety.check(temperature)

        if temperature is None:
            output.turn_off()
            led.update(safety.state, False)
            output.tick()
            continue

        # ── RL agent decides duty level ───────────────────────
        duty = agent.step(temperature, TARGET_TEMP, safety.blocked, dt)

        # ── Apply output ──────────────────────────────────────
        if output_allowed:
            output.set_duty(duty)
        else:
            output.turn_off()

        output.tick()

        is_active = output.current_duty > 0
        led.update(safety.state, is_active)

        # ── Update display ────────────────────────────────────
        if display is not None and loop_count % DISPLAY_UPDATE_EVERY == 0:
            try:
                err = TARGET_TEMP - temperature
                display.update(
                    temperature  = temperature,
                    setpoint     = TARGET_TEMP,
                    pid_output   = output.current_duty,
                    p_term       = agent.epsilon,   # repurposed: show ε
                    i_term       = float(agent.steps),
                    d_term       = 0.0,
                    kp           = RL_ALPHA,
                    ki           = RL_GAMMA,
                    kd           = 0.0,
                    safety_state = safety.state,
                    tune_count   = agent.steps // RL_SAVE_EVERY,
                    last_action  = "e={:.3f}".format(agent.epsilon),
                    loop_count   = loop_count,
                )
            except Exception as e:
                print("[DISPLAY] update error:", e)

        # ── Serial log ────────────────────────────────────────
        if loop_count % LOG_EVERY_LOOPS == 0:
            err = TARGET_TEMP - temperature
            print("T={:.2f}C  SP={:.1f}C  Err={:+.2f}  Duty={:.0f}%  "
                  "Safety={}  e={:.3f}  steps={}".format(
                temperature, TARGET_TEMP, err, output.current_duty,
                safety.state, agent.epsilon, agent.steps))
            if SERIAL_PRINT:
                _log("{},{:.2f},{:.1f},{:.2f},{:.0f},{},{:.3f},{}".format(
                    time.ticks_ms(), temperature, TARGET_TEMP, err,
                    output.current_duty, safety.state,
                    agent.epsilon, agent.steps))


try:
    main()
except KeyboardInterrupt:
    print("\nStopped. Output off.")
    Pin(PIN_OUTPUT, Pin.OUT).value(0)
