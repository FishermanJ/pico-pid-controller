# ESP32 / Pico Temperature Controller — MicroPython

MicroPython temperature controller with PID auto-tuning and a Reinforcement Learning (Q-learning) variant.
Works on **ESP32** and **Raspberry Pi Pico** (RP2040).

---

## Files

| File | Board | Description |
|---|---|---|
| `esp32_pid_controller.py` | ESP32 | PID controller with auto-tuner — original version |
| `pico_pid_controller.py` | Pico | Same PID controller ported to Pico |
| `pico_pid_v2_rl.py` | Pico | **v2** — Q-learning agent replaces PID |
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

## Wiring

### Pico (v1 and v2)

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
- `display_oled.py` — only if using OLED display (included in this repo)
- No external libraries needed for v1 or v2
