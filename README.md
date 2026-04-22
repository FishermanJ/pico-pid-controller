# ESP32 / Pico Temperature Controller — MicroPython

MicroPython temperature controller with PID auto-tuning, a Reinforcement Learning (Q-learning) variant, best-gains persistence, and a 3-button OLED menu UI.
Works on **ESP32** and **Raspberry Pi Pico** (RP2040).

---

## Files

| File | Board | Description |
|---|---|---|
| `esp32_pid_controller.py` | ESP32 | PID controller with auto-tuner — original version |
| `pico_pid_controller.py` | Pico | **v1** — PID controller ported to Pico, soft-PWM |
| `pico_pid_v2_rl.py` | Pico | **v2** — Q-learning agent replaces PID |
| `pico_pid_v3_smart.py` | Pico | **v3** — RUN / CALIBRATE modes, best-gains persistence, button reset |
| `pico_pid_v4_menu.py` | Pico | **v4** — 3-button OLED menu UI for live parameter editing |
| `display_oled.py` | Pico | SSD1306 128×64 OLED driver (self-contained, no extra library needed) |

---

## Version History

### v1 — PID Controller (`esp32_pid_controller.py` / `pico_pid_controller.py`)

- PID control with anti-windup
- Auto-tuner — adjusts Kp / Ki / Kd based on live performance
- Three-level safety system (warning → cutoff → emergency shutdown)
- PWM or relay output mode
- Optional OLED / IPS color display
- Serial output (CSV or human-readable)

**Pico-specific changes vs ESP32 version:**
- Soft-PWM replaces hardware PWM — works at any frequency including 1 Hz (ESP32 hardware minimum is ~8 Hz)
- ADC uses `read_u16()` (16-bit, 0–65535) instead of `read()` (12-bit, 0–4095)
- No `ADC.ATTN_11DB` — Pico does not need attenuation
- I2C bus 1 used for OLED on GP14/GP15
- OLED driver is self-contained in `display_oled.py` — Pico firmware does not include `ssd1306` unlike ESP32

---

### v2 — Reinforcement Learning Controller (`pico_pid_v2_rl.py`)

Replaces the PID formula with a **Q-learning agent** that learns the optimal heater power level by trial and error.

**How it works:**

The agent observes two things each step:
- **Error** — how far the temperature is from the target (9 buckets)
- **Rate** — whether temperature is rising or falling (5 buckets, °C/s)

From those it picks one of 7 output levels: `[0, 15, 30, 50, 70, 85, 100]%`

It receives a reward each step:

| Situation | Reward |
|---|---|
| Within 1°C of target | +1.0 |
| Within 3°C | +0.2 |
| More than 3°C away | negative (proportional) |
| Safety system triggered | −10.0 |

Q-table size: 9 × 5 × 7 = **315 entries** — fits easily in Pico RAM.

The Q-table is **saved to flash** (`qtable_v2.json`) every 200 steps, so learning persists across reboots.

**Learning phases:**
1. First run: ε = 0.90 — mostly random exploration
2. After reboot with saved Q-table: ε = 0.05 — mostly exploits what it learned

---

### v3 — Smart PID with RUN / CALIBRATE modes (`pico_pid_v3_smart.py`)

Adds persistent best-gains tracking and two operating modes switchable via a single button.

**Modes:**

| Mode | Tuner aggressiveness | Tuner interval | Purpose |
|---|---|---|---|
| RUN | Gentle (step ±0.03) | Every 60 s | Stable daily operation |
| CALIBRATE | Aggressive (step ±0.10) | Every 20 s | Fast exploration of better gains |

**How it works:**
- A `PerformanceTracker` scores each tuning window: `score = MAE + 0.5 × oscillations/min` (lower is better)
- When a new score beats the stored best, gains are saved to `pid_state.json` on flash
- In RUN mode, if current score is >30% worse than the stored best, the best gains are automatically reloaded
- CALIBRATE mode runs for up to 5 minutes (`CAL_DURATION_S`), then returns to RUN automatically

**Button (GP17):**
- Short press — toggle RUN ↔ CALIBRATE
- Long press ≥ 3 s — factory reset (clears saved state, reloads defaults)

**LED patterns:**
- RUN — normal slow blink
- CALIBRATE — double-blink pattern

**Saved state (`pid_state.json`):**
```json
{"best_kp": 1.2, "best_ki": 0.04, "best_kd": 0.8, "best_score": 0.42, "mode": "RUN", "cal_count": 3}
```

---

### v4 — OLED Menu UI (`pico_pid_v4_menu.py`)

All v3 functionality plus a **3-button live menu** on the OLED display for editing parameters without reflashing.

**Buttons:**

| Button | Pin | Short press | Long press |
|---|---|---|---|
| SELECT | GP17 | Open menu / advance to next item | Save & exit menu (or factory reset outside menu) |
| UP | GP18 | Increase selected value | — |
| DOWN | GP19 | Decrease selected value | — |

UP / DOWN auto-repeat when held.

**Menu items:**

| Item | Step | Description |
|---|---|---|
| MODE | — | Toggle RUN ↔ CALIBRATE |
| TEMP | ±0.5 °C | Target temperature setpoint |
| Kp | ±0.1 | Proportional gain |
| Ki | ±0.005 | Integral gain |
| Kd | ±0.1 | Derivative gain |

**OLED layout while menu is open:**
```
[header: item name + value]   ← row 0
[item 1]                       ← row 1
[item 2]  ◄ highlighted        ← row 2  (white bg, black text)
[item 3]                       ← row 3
[item 4]                       ← row 4
[item 5]                       ← row 5
[HOLD SEL=SAVE / SEL=NEXT]    ← row 7 (hint)
```

LED is solid ON while the menu is open. All changes are applied immediately when saved.

---

## Wiring

### Pico (v1 / v2)

```
NTC thermistor (high-side):
  3.3V ──── NTC ──── GP26 (ADC0) ──── 100kΩ ──── GND

DS18B20 (alternative digital sensor):
  3.3V ── VCC
  GND  ── GND
  GP26 ── DATA ──── 4.7kΩ ──── 3.3V   (pull-up required)

Heater output:
  GP16 ──── relay / SSR / MOSFET IN

Built-in LED:
  GP25  (use Pin("LED") on Pico W)

OLED display (SSD1306 I2C):
  GP14 ──── SDA
  GP15 ──── SCL
```

### Pico (v3 / v4 — additional pins)

```
Button – SELECT (v3 mode toggle / v4 menu navigate):
  GP17 ──── button ──── GND   (internal pull-up, active LOW)

Button – UP (v4 only):
  GP18 ──── button ──── GND

Button – DOWN (v4 only):
  GP19 ──── button ──── GND
```

All buttons connect directly between the GPIO pin and GND — no external resistor needed (internal pull-ups are enabled in software).

### ESP32

```
NTC thermistor:
  3.3V ──── NTC ──── GPIO34 ──── 10kΩ ──── GND

DS18B20:
  GPIO33 ── DATA ──── 4.7kΩ ──── 3.3V   (pull-up required)

Output:
  GPIO25 ──── relay IN

OLED (I2C):
  GPIO21 ──── SDA
  GPIO22 ──── SCL
```

---

## NTC Thermistor Values

Change these three settings to match your thermistor:

| Setting | 10k NTC | 100k NTC |
|---|---|---|
| `NTC_R_PULLUP` / `NTC_R_SERIES` | `10_000` | `100_000` |
| `NTC_R_AT_25C` | `10_000` | `100_000` |
| `NTC_B_VALUE` | datasheet (typically 3950) | datasheet (typically 3950) |

Use a series resistor matching your NTC value (10k with 10k NTC, 100k with 100k NTC) for best accuracy.

**NTC orientation:**
- NTC on **high side** (between 3.3V and ADC): formula uses `(Vin − V) / V` — default in Pico files
- NTC on **low side** (between ADC and GND): formula uses `V / (Vin − V)` — default in ESP32 file

---

## Quick Start

### v1 PID (Pico)
1. Upload `pico_pid_controller.py` (rename to `main.py` on the Pico) and `display_oled.py`
2. Edit **Section 1 — SETTINGS**: set `TARGET_TEMP`, NTC values, pins
3. Set `DISPLAY_TYPE = "OLED"` if you have a display
4. Open Serial Monitor at **115200 baud**

### v2 RL (Pico)
1. Upload `pico_pid_v2_rl.py` (rename to `main.py`) and `display_oled.py`
2. Set `TARGET_TEMP` and NTC values in Section 1
3. First run will explore randomly — takes 15–30 minutes to build a useful policy
4. Subsequent runs load `qtable_v2.json` from flash and exploit the learned policy

### v3 Smart PID (Pico)
1. Upload `pico_pid_v3_smart.py` (rename to `main.py`) and `display_oled.py`
2. Set `TARGET_TEMP` and NTC values in Section 1
3. Wire a button between **GP17** and **GND**
4. First run starts in RUN mode with default PID gains
5. Short-press the button to enter CALIBRATE mode — let it run for a few minutes to find better gains
6. Best gains are saved automatically; a reboot reloads them

### v4 OLED Menu (Pico)
1. Upload `pico_pid_v4_menu.py` (rename to `main.py`) and ensure `display_oled.py` is also on the Pico
2. Set `TARGET_TEMP` and NTC values in Section 1
3. Wire three buttons: **GP17** (SELECT), **GP18** (UP), **GP19** (DOWN) — each to GND
4. Press SELECT to open the menu, UP/DOWN to change values, long-press SELECT to save and exit
5. All changes take effect immediately; setpoint and gains persist across reboots

---

## Key Settings — v1 PID

| Setting | Default | Description |
|---|---|---|
| `TARGET_TEMP` | `60.0` | Target temperature in °C |
| `SENSOR_TYPE` | `"NTC"` | `"NTC"` or `"DS18B20"` |
| `RELAY_MODE` | `False` | `True` = on/off relay, `False` = soft-PWM |
| `PWM_FREQUENCY_HZ` | `1` | Soft-PWM cycle frequency (any value, no hardware limit on Pico) |
| `AUTOTUNER_ENABLED` | `True` | Auto-adjust PID gains while running |
| `DISPLAY_TYPE` | `"NONE"` | `"OLED"`, `"IPS"`, or `"NONE"` |
| `SERIAL_FORMAT` | `"CSV"` | `"CSV"` (Serial Plotter) or `"HUMAN"` |

## Key Settings — v2 RL

| Setting | Default | Description |
|---|---|---|
| `TARGET_TEMP` | `60.0` | Target temperature in °C |
| `OUTPUT_LEVELS` | `[0,15,30,50,70,85,100]` | Duty cycle choices available to the agent |
| `RL_ALPHA` | `0.15` | Learning rate — how fast Q-values update |
| `RL_GAMMA` | `0.92` | Discount — how much future rewards matter |
| `RL_EPSILON_START` | `0.90` | Initial exploration rate |
| `RL_EPSILON_MIN` | `0.05` | Minimum exploration rate |
| `RL_SAVE_EVERY` | `200` | Save Q-table to flash every N steps |
| `QTABLE_FILE` | `"qtable_v2.json"` | Flash file where learning is stored |

## Key Settings — v3 Smart PID

| Setting | Default | Description |
|---|---|---|
| `TARGET_TEMP` | `60.0` | Target temperature in °C |
| `PIN_BUTTON` | `17` | GP pin for mode-toggle / reset button |
| `CAL_DURATION_S` | `300` | Seconds before CALIBRATE auto-returns to RUN |
| `SCORE_DEADBAND` | `1.0` | °C deadband used in oscillation counting |
| `TUNE_STEP_RUN` | `0.03` | Gain adjustment step in RUN mode |
| `TUNE_STEP_CAL` | `0.10` | Gain adjustment step in CALIBRATE mode |
| `TUNE_INTERVAL_RUN` | `60` | Seconds between tune evaluations in RUN mode |
| `TUNE_INTERVAL_CAL` | `20` | Seconds between tune evaluations in CALIBRATE mode |
| `STATE_FILE` | `"pid_state.json"` | Flash file for best gains and mode |

## Key Settings — v4 OLED Menu

Inherits all v3 settings, plus:

| Setting | Default | Description |
|---|---|---|
| `PIN_SELECT` | `17` | GP pin for SELECT button (menu open / next / save) |
| `PIN_UP` | `18` | GP pin for UP button |
| `PIN_DOWN` | `19` | GP pin for DOWN button |
| `LONG_MS` | `1000` | Hold time (ms) to trigger long-press |
| `REPEAT_MS` | `300` | Auto-repeat interval (ms) for UP/DOWN when held |

---

## Serial Output

**CSV** (paste into Serial Plotter or Excel):
```
timestamp_ms,temperature_C,setpoint_C,error,duty_pct,safety,epsilon,steps
12500,58.32,60.0,-1.68,50,OK,0.312,240
```

**Status line** (printed every 10 loops):
```
T= 58.32°C  SP=60.0°C  Err=-1.68  Out= 50.0%  Safety=OK  e=0.312  steps=240
```

---

## Safety System

Same in both v1 and v2 — safety always overrides the controller.

| Level | Trigger | Action |
|---|---|---|
| WARNING | setpoint + WARN_PCT% | Log only, output continues |
| CUTOFF | setpoint + CUTOFF_PCT% | Output off, PID/agent reset |
| SHUTDOWN | `SAFETY_HARD_MAX_C` | Emergency off, stays off until reboot |

In v2 the agent also receives a **−10 reward** when safety triggers, so it learns to avoid overheating.
In v3 and v4, a safety trigger also resets the performance score so bad gains are never saved as "best".

---

## OLED Display Setup (Pico)

`display_oled.py` is self-contained — the SSD1306 driver is included inside the file.
No extra library installation needed.

1. Upload `display_oled.py` to the root of the Pico filesystem
2. Set `DISPLAY_TYPE = "OLED"` in your controller file
3. Verify I2C address with the REPL:
```python
from machine import I2C, Pin
print(I2C(1, sda=Pin(14), scl=Pin(15)).scan())
# [60] = 0x3C  (default)
# [61] = 0x3D  → change OLED_I2C_ADDR = 0x3D
```

---

## Requirements

- MicroPython firmware on ESP32 or Pico
- `onewire` + `ds18x20` — only if using DS18B20 (built into standard firmware)
- `display_oled.py` — required for v3 and v4; optional for v1/v2 (included in this repo)
- No external libraries needed for any version
