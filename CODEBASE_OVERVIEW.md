# CODEBASE_OVERVIEW.md — YourWingC3 Flight Controller

> **Generated for AI handoff.** This document fully describes the YourWingC3 firmware so another assistant can continue development without reading every source file. All values were verified against the current source code as of this writing.

---

## 1. Project Summary

**YourWingC3** is a lightweight flight controller firmware for micro quadcopters built on the **ESP32-C3 Super Mini** (single-core RISC-V, 160 MHz). It runs a 250 Hz cascaded PID control loop, reads an MPU6050 IMU over I2C, drives four 0716 coreless DC brushed motors via MOSFETs with LEDC PWM, and exposes a WiFi Access Point with a browser-based dual-joystick controller and real-time telemetry over WebSockets. Designed for hobbyists building indoor micro brushed quads.

**Note on branding:** The project has been rebranded from "Comet Drone" to "YourWingC3", but the source code still contains the old "Comet Drone" name in several files (see Section 12 for details). The README.md and this document use the new name.

---

## 2. Tech Stack & Build System

| Item | Detail |
|---|---|
| **Framework** | Arduino (Espressif ESP32 Arduino Core) |
| **Board target** | `ESP32C3 Dev Module` (select in Arduino IDE) |
| **Language** | C++ (.ino, .cpp, .h) + embedded HTML/JS/CSS |
| **Build tool** | Arduino IDE (no PlatformIO / CMake — flat `.ino` sketch) |
| **Required library** | `WebSockets` by Links2004 (Arduino Library Manager) |
| **Board package** | `esp32` by Espressif Systems (Boards Manager) |
| **Serial baud** | 115200 |
| **Key board settings** | USB CDC On Boot: Enabled; Flash Mode: QIO or DIO |

No FreeRTOS task spawning, no ESP-IDF direct calls — everything runs cooperatively inside the Arduino `loop()`.

---

## 3. Folder & File Structure

```
YourWingC3/
├── drone_firmware.ino   # Main sketch: setup(), loop(), serial CLI, PID init
├── config.h             # ALL tunable constants: pins, PID gains, WiFi, safety
├── imu.h / imu.cpp     # MPU6050 driver: I2C init, raw read, complementary filter
├── pid.h / pid.cpp      # Generic PID controller class (anti-windup, D-on-measurement)
├── motors.h / motors.cpp# Motor init (LEDC PWM), X-quad mixing, dynamic scaling
├── battery.h / battery.cpp # 1S LiPo ADC voltage reader with moving-average filter
├── wifi_control.h / wifi_control.cpp # WiFi AP, HTTP server, WebSocket server, command parse
├── web_page.h           # PROGMEM string: full single-page HTML/CSS/JS controller UI
├── README.md            # User-facing docs (setup, wiring, serial commands)
├── LICENSE              # MIT License (copyright: Bhushan Patil)
└── CODEBASE_OVERVIEW.md # This file — full technical documentation
```

---

## 4. Core Architecture

### Single-Thread Cooperative Loop (No RTOS Tasks)

Everything runs in one Arduino `loop()` at a fixed **250 Hz** (4 ms period). The loop busy-waits using `yield()` (critical on the single-core ESP32-C3 to avoid starving the WiFi stack).

```
┌──────────────────────────────────────────────────────┐
│  setup()  (drone_firmware.ino:92)                     │
│  1. Serial.begin(115200)                              │
│  2. LED init (pinMode OUTPUT)                         │
│  3. IMU.begin() + IMU.calibrate()  ← blocks ~1s      │
│  4. Motors.begin() + Motors.stop()                    │
│  5. Battery monitoring printed as DISABLED            │
│  6. initPIDs() — configure all 5 PID controllers     │
│  7. WiFi AP + HTTP + WebSocket start                  │
│  8. Timing init                                       │
└──────────────────────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│  loop()  (drone_firmware.ino:158) @ 250 Hz           │
│                                                      │
│  0. yield()-wait for 4 ms boundary                   │
│  1. IMU.update(dt) → get roll/pitch/yaw + rates      │
│  2. WiFi.update() → parse latest command             │
│  3. Failsafe check (signal timeout → disarm)         │
│  4. Arm/disarm logic                                 │
│  5. PID compute (angle→rate cascade or direct rate)  │
│  6. Motor mixing → LEDC PWM write                    │
│  7. Telemetry broadcast (every 200 ms)               │
│  8. Loop frequency counter                           │
│  9. Status LED pattern                               │
│ 10. Serial command handler                           │
└──────────────────────────────────────────────────────┘
```

### Inter-Module Communication

Modules are **not coupled** — the main `.ino` loop acts as the orchestrator, pulling data from each module and pushing results to the next. Data structures (`IMUData`, `ControlCommand`) serve as the interface. There are no callbacks into flight logic; WebSocket callbacks only update a static `ControlCommand` struct.

---

## 5. Key Modules Breakdown

### 5.1 IMU (`imu.h` / `imu.cpp`)

**Purpose:** Read MPU6050 accelerometer + gyroscope, apply calibration offsets, and fuse into orientation angles using a complementary filter.

**Key functions:**
| Function | What it does |
|---|---|
| `begin()` | I2C init via `Wire.begin()` (no arguments — uses ESP32-C3 defaults: GPIO 8 SDA, GPIO 9 SCL). MPU6050 reset, config DLPF (44 Hz), gyro ±500°/s, accel ±4g, 1 kHz sample rate. |
| `calibrate()` | Collects 2000 readings while stationary. Computes mean gyro offsets and accel offsets (removes 1g from Z). Initializes roll/pitch from accel. |
| `update(dt)` | Reads raw, subtracts offsets, computes accel angles, applies complementary filter: `0.98 * (gyro_integral) + 0.02 * (accel_angle)`. Yaw is gyro-only (no mag). |
| `getData()` | Returns `IMUData` struct by value. |

**Dependencies:** `Wire.h` (I2C), `config.h`

### 5.2 PID Controller (`pid.h` / `pid.cpp`)

**Purpose:** Generic, reusable PID controller with anti-windup clamping and D-on-measurement (avoids derivative kick on setpoint changes).

**Key functions:**
| Function | What it does |
|---|---|
| `compute(setpoint, measurement, dt)` | Returns constrained PID output. P on error, I with clamped integral, D on measurement derivative. |
| `reset()` | Zeros integral, prev measurement, first-run flag. Called on arm/disarm. |
| `setKp/Ki/Kd(v)` | Runtime gain adjustment (used by serial CLI). |

**Internal design notes:**
- D-term: `derivative = -(measurement - prevMeasurement) / dt` (negation = D on measurement)
- Integral clamped to `PID_INTEGRAL_MAX` (50.0)
- Output clamped to configured `_minOut` / `_maxOut`
- `_firstRun` flag prevents D-spike on first `compute()` call

**Dependencies:** `Arduino.h` (for `constrain()`)

### 5.3 Motors (`motors.h` / `motors.cpp`)

**Purpose:** Initialize LEDC PWM channels, perform X-quad motor mixing, enforce safety limits, and write PWM values.

**Key functions:**
| Function | What it does |
|---|---|
| `begin()` | Attaches 4 pins to LEDC at 20 kHz / 8-bit resolution. |
| `mix(throttle, roll, pitch, yaw)` | Computes per-motor values using X-quad formulas, applies dynamic overflow scaling, enforces idle floor, writes PWM. |
| `stop()` | All motors to 0. |
| `testMotor(index, speed)` | Spins one motor for bench testing. |

**X-Quad Mixing Formulas:**
```
M1 (FR, CCW) = Throttle - Roll - Pitch + Yaw
M2 (RL, CCW) = Throttle + Roll + Pitch + Yaw
M3 (FL, CW)  = Throttle + Roll - Pitch - Yaw
M4 (RR, CW)  = Throttle - Roll + Pitch - Yaw
```

**Dynamic scaling:** If any motor exceeds `MOTOR_MAX`, all motors are shifted down by the overflow amount (preserves PID correction ratios). If throttle > `MOTOR_IDLE` (20), a minimum floor of 20 is enforced.

**Dependencies:** `config.h` (pin definitions, PWM constants)

### 5.4 Battery (`battery.h` / `battery.cpp`)

**Purpose:** Read 1S LiPo voltage via ADC through a voltage divider, apply a 10-sample moving average, and set warning/critical flags.

**Status: DISABLED in main sketch.** The `#include "battery.h"` and `Battery battery` object are commented out in `drone_firmware.ino` (lines 23, 29). No voltage divider is physically connected. The class is fully implemented but unused.

**Key constants:** `BATTERY_DIVIDER = 2.0`, `BATTERY_FULL = 4.20V`, `BATTERY_EMPTY = 3.00V`, `BATTERY_WARNING = 3.30V`, `BATTERY_CRITICAL = 3.00V`, 12-bit ADC.

### 5.5 WiFi Control (`wifi_control.h` / `wifi_control.cpp`)

**Purpose:** Manage WiFi Access Point, serve the web controller page over HTTP, handle WebSocket connections for bidirectional command/telemetry.

**Key functions:**
| Function | What it does |
|---|---|
| `begin()` | Starts AP (SSID from `WIFI_SSID` config, channel 6, 19.5 dBm TX), disables modem sleep, HTTP on port 80, WebSocket on port 81. |
| `update()` | Calls `httpServer.handleClient()` and `wsServer.loop()`. |
| `sendTelemetry(...)` | Broadcasts JSON to all WS clients every 200 ms. |
| `onWebSocketEvent()` (static) | Connect/disconnect/text callbacks. On disconnect: auto-disarm + zero throttle. |
| `parseCommand()` (static) | Parses CSV-like string `"T:0.50,R:0.00,P:0.00,Y:0.00,A:1,M:0"` into `ControlCommand`. |

**Command protocol (controller → drone):**
```
T:<throttle>,R:<roll>,P:<pitch>,Y:<yaw>,A:<armed>,M:<mode>
```
- Throttle: 0.0–1.0; Roll/Pitch/Yaw: -1.0 to 1.0; Armed: 0 or 1; Mode: 0 (angle) or 1 (rate)

**Telemetry protocol (drone → controller, JSON):**
```json
{"r":0.0,"p":0.0,"y":0.0,"bv":0.00,"bp":0,"a":0,"hz":250,"m1":0,"m2":0,"m3":0,"m4":0}
```

**Important static state:** `_cmd`, `_connected`, `_lastCmdTime` are all static class members (singleton pattern).

**Dependencies:** `WiFi.h`, `WebServer.h`, `WebSocketsServer.h`, `config.h`, `web_page.h`

### 5.6 Web Page (`web_page.h`)

**Purpose:** Single-file HTML/CSS/JS web application stored as a `PROGMEM` string. Served at `http://192.168.4.1/`.

**Features:**
- Dual virtual joysticks (left = throttle/yaw, right = roll/pitch) with touch + mouse support
- Left stick Y (throttle) does **not** auto-center on release; X (yaw) does
- Right stick auto-centers both axes on release
- ARM / DISARM / STOP buttons
- ANGLE / RATE mode toggle
- Live telemetry: roll, pitch, yaw angles + battery percentage
- Motor output bar indicators (FL, FR, RL, RR)
- Command send rate: **20 Hz** (`setInterval` every 50 ms)

**Current branding:** Title says "Comet Drone", logo says "COMET DRONE" (old branding — not yet updated to YourWingC3).

---

## 6. Data Flow

```
MPU6050 (I2C @ 100kHz, GPIO 8/9)
    │
    ▼
IMU.readRaw()          ← 14 bytes: accel XYZ + temp + gyro XYZ
    │
    ▼
IMU.update(dt)         ← subtract calib offsets → complementary filter
    │                     → _roll, _pitch, _yaw, _rollRate, _pitchRate, _yawRate
    ▼
IMUData struct         ← returned by value via IMU.getData()
    │
    ├──────► [Failsafe check] ──signal timeout?──► DISARM
    │
    ├──────► [Arm/disarm gate] ──armed?──► continue
    │
    ▼
Scale cmd inputs       ← cmd.throttle × 255, cmd.roll × 45°, etc.
    │
    ▼
PID Cascade (Angle mode):
    ├─ rollAnglePID(desired_angle, imu.roll, dt)  → desired_roll_rate
    ├─ pitchAnglePID(desired_angle, imu.pitch, dt) → desired_pitch_rate
    ├─ rollRatePID(desired_rate, imu.rollRate, dt) → motor_correction_roll
    ├─ pitchRatePID(desired_rate, imu.pitchRate, dt) → motor_correction_pitch
    └─ yawRatePID(desired_rate, imu.yawRate, dt) → motor_correction_yaw
    │
    ▼
Motor.mix(throttle, roll, pitch, yaw)
    │  ← X-quad formulas → 4 motor values
    │  ← dynamic overflow scaling
    │  ← clamp to [MOTOR_IDLE..MOTOR_MAX]
    ▼
ledcWrite() × 4        ← 20 kHz PWM to MOSFETs → motors spin
```

**WebSocket data flow (parallel):**
```
Phone Browser ──WS(20Hz)──► parseCommand() ──► _cmd struct ──► main loop reads
                                    ▲
Main loop ──every 200ms──► sendTelemetry() ──JSON broadcast──► Phone Browser
```

---

## 7. Pin Mapping / Hardware Config

### Verified Pin Assignments (from current source code)

| Function | GPIO | Source | Peripheral |
|---|---|---|---|
| Motor FR (M1, CCW) | **GPIO 0** | `config.h:12` | LEDC PWM, 20 kHz |
| Motor FL (M3, CW) | **GPIO 1** | `config.h:14` | LEDC PWM, 20 kHz |
| Motor RL (M2, CCW) | **GPIO 3** | `config.h:13` | LEDC PWM, 20 kHz |
| Motor RR (M4, CW) | **GPIO 4** | `config.h:15` | LEDC PWM, 20 kHz |
| Battery ADC | **GPIO 6** | `config.h:16` | ADC (12-bit, currently unused) |
| Status LED | **GPIO 8** | `config.h:17` | Digital output, active LOW |
| I2C SDA (MPU6050) | **GPIO 8** | `imu.cpp:5` (`Wire.begin()` no args = ESP32-C3 default) | I2C data |
| I2C SCL (MPU6050) | **GPIO 9** | `imu.cpp:5` (`Wire.begin()` no args = ESP32-C3 default) | I2C clock |

### Pin Conflict: GPIO 8 (LED + I2C SDA)

**This is a real, unresolved hardware conflict.** Both the status LED and I2C SDA are on GPIO 8:

- `config.h:17`: `#define PIN_LED 8`
- `imu.cpp:5`: `Wire.begin()` — no arguments, uses ESP32-C3 default SDA = GPIO 8
- `drone_firmware.ino:103`: `pinMode(PIN_LED, OUTPUT)` — drives GPIO 8 as push-pull
- `drone_firmware.ino:48,49,269,271,273`: `digitalWrite(PIN_LED, ...)` — drives GPIO 8 HIGH/LOW

The LED is driven as a push-pull output while I2C SDA requires open-drain signaling. This will corrupt I2C transactions if LED updates and I2C reads overlap. It works by timing luck (LED at loop end, I2C at loop start) but is fragile. **Fix:** Move LED to a free GPIO (e.g. GPIO 2) by changing `PIN_LED` in `config.h`.

---

## 8. Configuration & Tuning Parameters

All constants in **`config.h`**. No EEPROM / persistent storage — values reset on every boot.

### PID Gains (Angle Loop — Outer)
| Parameter | Constant | Default |
|---|---|---|
| Roll Angle Kp/Ki/Kd | `ROLL_ANGLE_KP/KI/KD` | 2.0 / 0.5 / 0.1 |
| Pitch Angle Kp/Ki/Kd | `PITCH_ANGLE_KP/KI/KD` | 2.0 / 0.5 / 0.1 |

### PID Gains (Rate Loop — Inner)
| Parameter | Constant | Default |
|---|---|---|
| Roll Rate Kp/Ki/Kd | `ROLL_RATE_KP/KI/KD` | 0.5 / 0.1 / 0.03 |
| Pitch Rate Kp/Ki/Kd | `PITCH_RATE_KP/KI/KD` | 0.5 / 0.1 / 0.03 |
| Yaw Rate Kp/Ki/Kd | `YAW_RATE_KP/KI/KD` | 0.8 / 0.15 / 0.0 |

### PID Limits
| Constant | Default | Description |
|---|---|---|
| `PID_MAX_OUTPUT` | 120.0 | Max motor correction |
| `PID_INTEGRAL_MAX` | 50.0 | Anti-windup integral clamp |
| `MAX_ANGLE` | 45.0° | Maximum tilt in angle mode |
| `MAX_YAW_RATE` | 180.0°/s | Maximum yaw rotation speed |

### Control Loop
| Constant | Default |
|---|---|
| `LOOP_FREQUENCY` | 250 Hz |
| `LOOP_TIME_US` | 4000 µs |

### Motor Settings
| Constant | Default |
|---|---|
| `PWM_FREQUENCY` | 20000 Hz |
| `PWM_RESOLUTION` | 8 (0–255) |
| `MOTOR_MAX` | 255 |
| `MOTOR_MIN` | 0 |
| `MOTOR_IDLE` | 20 |
| `THROTTLE_HEADROOM` | 80 |

### IMU Settings
| Constant | Default |
|---|---|
| `MPU6050_ADDR` | 0x68 |
| `GYRO_SENSITIVITY` | 65.5 LSB/(°/s) at ±500°/s |
| `ACCEL_SENSITIVITY` | 8192.0 LSB/g at ±4g |
| `COMPLEMENTARY_ALPHA` | 0.98 |
| `CALIBRATION_SAMPLES` | 2000 |

### WiFi Settings
| Constant | Default |
|---|---|
| `WIFI_SSID` | `"CometDrone"` (**not yet rebranded to YourWingC3 in code**) |
| `WIFI_PASSWORD` | `"drone1234"` |
| `WIFI_CHANNEL` | 6 |
| `WS_PORT` | 81 |
| `HTTP_PORT` | 80 |

### Safety
| Constant | Default |
|---|---|
| `SIGNAL_TIMEOUT` | 500 ms |
| `TELEMETRY_INTERVAL` | 200 ms |

### Battery (unused)
| Constant | Default |
|---|---|
| `BATTERY_DIVIDER` | 2.0 |
| `BATTERY_FULL` | 4.20 V |
| `BATTERY_EMPTY` | 3.00 V |
| `BATTERY_WARNING` | 3.30 V |
| `BATTERY_CRITICAL` | 3.00 V |

### Serial CLI Commands
Format: `<Axis><Param>:<Value>` — e.g. `RP:0.7`, `AI:0.2`
- `RP/RI/RD` — Roll rate Kp/Ki/Kd
- `PP/PI/PD` — Pitch rate Kp/Ki/Kd
- `YP/YI/YD` — Yaw rate Kp/Ki/Kd
- `AP/AI/AD` — Roll angle Kp/Ki/Kd
- `BP/BI/BD` — Pitch angle Kp/Ki/Kd
- `MT:<0-3>` — Test motor at speed 80
- `MS` — Emergency stop all motors
- `ST` — Print full status

---

## 9. Known Issues / TODOs / Incomplete Features

### Issues Introduced by Comet-Drone Clone (Reverted Previous Fixes)

1. **WiFi SSID still says "CometDrone"** — `config.h:68` still has `#define WIFI_SSID "CometDrone"`. The rename to "YourWingC3" was done previously but the Comet-Drone clone overwrote `config.h`.

2. **Pin wiring comment is wrong** — `drone_firmware.ino:13-14` still says `Motor RL → GPIO1` and `Motor FL → GPIO3` and `Motor RR → GPIO10`. The actual `config.h` values are RL=3, FL=1, RR=4. This was fixed before but reverted by the clone.

3. **Dead `else if (false)` branch** — `drone_firmware.ino:270-271` still has the unreachable battery warning LED branch. Was previously removed but reverted by the clone.

4. **"COMET DRONE" in code comments and serial banner** — `config.h:5`, `drone_firmware.ino:2`, `drone_firmware.ino:98` all still say "COMET DRONE". The rebrand was previously applied but reverted by the clone.

5. **Web page still says "Comet Drone"** — `web_page.h:12` (title) and `web_page.h:97` (logo div) still say "COMET DRONE".

6. **LICENSE says "Bhushan Patil"** — `LICENSE:3` has `Copyright (c) 2026 Bhushan Patil` (from the Comet-Drone repo). Was previously "YourWingC3" but overwritten.

### Pre-existing / Unchanged Issues

7. **GPIO 8 pin conflict (LED + I2C SDA)** — Unresolved. Both on GPIO 8 (see Section 7). Move LED to GPIO 2.

8. **No EEPROM / persistent storage** — All PID gains reset on reboot.

9. **Yaw drift** — Yaw is gyro-only integrated (no magnetometer), drifts continuously.

10. **Single-threaded WiFi + flight control** — `yield()` workaround, but jitter possible under heavy WiFi load.

11. **No DShot/ESC protocol** — Brushed DC PWM only.

12. **Battery monitoring disabled** — Code exists but commented out (no voltage divider wired).

13. **Web page loads Google Fonts from CDN** — Will fail if phone has no internet while on drone AP.

14. **LICENSE file author** — Currently "Bhushan Patil" (from Comet-Drone repo), should likely be updated.

---

## 10. Entry Points

### `setup()` — `drone_firmware.ino:92`
Initialization order:
1. Serial at 115200
2. LED pin as OUTPUT (`PIN_LED` = GPIO 8)
3. `imu.begin()` — I2C + MPU6050 register config (halts with error blink if I2C fails)
4. `imu.calibrate()` — 2000-sample gyro/accel offset calibration (drone must be still)
5. `motors.begin()` + `motors.stop()` — LEDC attach, all PWM to 0
6. `initPIDs()` — configure all 5 PID controllers with gains and limits
7. `wifiCtrl.begin()` — WiFi AP + HTTP + WebSocket servers
8. Print ready banner with SSID/password/IP
9. Initialize `loopTimer`, `hzTimer`, `lastTelemetry`

### `loop()` — `drone_firmware.ino:158`
Called repeatedly by Arduino runtime. Runs the full 250 Hz control cycle as described in Section 4.

### `handleSerialCommands()` — `drone_firmware.ino:292`
Called at the end of every `loop()` iteration. Non-blocking; only processes if `Serial.available()`.

---

## 11. Glossary of Project-Specific Terms

| Term | Meaning |
|---|---|
| **YourWingC3** | Project codename (rebranded from "Comet Drone") |
| **0716 motors** | 7 mm diameter × 16 mm length coreless brushed DC motors |
| **X-Quad** | Frame configuration where motors form an X shape; affects mixing formulas |
| **CCW / CW** | Counter-clockwise / Clockwise motor spin direction |
| **FR / FL / RL / RR** | Front-Right, Front-Left, Rear-Left, Rear-Right motor positions |
| **M1–M4** | Motor indices: M1=FR, M2=RL, M3=FL, M4=RR |
| **Angle mode** | Self-leveling mode (flightMode=0): stick position = desired tilt angle |
| **Rate mode** | Acro/manual mode (flightMode=1): stick position = desired rotation rate |
| **Cascaded PID** | Two-layer PID: outer angle loop → inner rate loop → motor correction |
| **D-on-measurement** | Derivative computed on measurement change (not error) to avoid derivative kick |
| **Complementary filter** | Sensor fusion: `0.98 * gyro + 0.02 * accel` |
| **Anti-windup** | Integral clamping to prevent I-term accumulation during saturation |
| **Dynamic scaling** | When any motor overflows, all shift down proportionally |
| **THROTTLE_HEADROOM** | PWM range reserved for PID corrections above commanded throttle |
| **MOTOR_IDLE** | Minimum PWM (20/255) when armed — keeps motors spinning |
| **Failsafe** | Auto-disarm if no WebSocket command for 500 ms |
| **LEDC** | ESP32's LED Control peripheral — used for PWM motor output |
| **DLPF** | Digital Low-Pass Filter — MPU6050 hardware filter at ~44 Hz |
| **PROGMEM** | Arduino macro to store data in flash memory (web page HTML) |
| **`yield()`** | Feeds RTOS idle task — essential on single-core ESP32-C3 |
| **`_firstRun`** | PID flag preventing D-spike on first `compute()` call |

---

## 12. Recent Changes Log

This section documents what has changed since the previous version of this overview.

### Completed Changes
1. **Project rebrand** — "Comet Drone" → "YourWingC3" applied to README.md and CODEBASE_OVERVIEW.md. **However, the Comet-Drone repo clone reverted the rebrand in source files** (config.h, drone_firmware.ino, web_page.h still say "Comet Drone"/"COMET DRONE").
2. **README.md fully rewritten** — Complete rewrite with verified pin table, serial CLI commands, GPIO 8 conflict warning, hardware requirements, getting started guide.
3. **LICENSE populated** — Was empty, now has MIT License text. Copyright holder is "Bhushan Patil" (from Comet-Drone repo).
4. **GPIO 8 pin conflict investigated** — Confirmed real conflict: LED (push-pull output) and I2C SDA (open-drain) both on GPIO 8. Documented in README with fix suggestion (move LED to GPIO 2). **Not yet fixed in code.**

### Reverted / Unresolved (Due to Comet-Drone Clone Overwrite)
5. **Pin wiring comment in drone_firmware.ino** — Was fixed (corrected GPIO numbers), now reverted to wrong values (GPIO1/GPIO3/GPIO10).
6. **Dead `else if (false)` branch** — Was removed, now reverted (still at line 270-271).
7. **WiFi SSID rename** — Was changed to "YourWingC3", now reverted to "CometDrone" in config.h.
8. **"COMET DRONE" in comments/banner** — Was changed to "YOURWINGC3", now reverted in config.h and drone_firmware.ino.
9. **Web page branding** — Was changed to "YourWingC3", now reverted to "Comet Drone" in web_page.h.

### Not Yet Done
10. **GPIO 8 conflict fix** — LED needs to be moved to GPIO 2 in config.h (PIN_LED change + physical rewiring).
11. **Complete rebrand of source files** — config.h, drone_firmware.ino, web_page.h still need "Comet Drone" → "YourWingC3" updates.
12. **Pin comment fix** — drone_firmware.ino:13-14 needs correction to match config.h.
13. **Dead branch removal** — drone_firmware.ino:270-271 `else if (false)` needs removal.
