# CODEBASE_OVERVIEW.md — YourWingC3 Flight Controller

> **Generated for AI handoff.** This document fully describes the YourWingC3 firmware so another assistant can continue development without reading every source file.

---

## 1. Project Summary

**YourWingC3** is a lightweight flight controller firmware for **micro-quadcopters** built on the **ESP32-C3 Super Mini** (single-core RISC-V, 160 MHz). It runs a 250 Hz cascaded PID control loop, reads an MPU6050 IMU over I2C, drives four 0716 coreless DC brushed motors via MOSFETs with LEDC PWM, and exposes a WiFi Access Point with a browser-based dual-joystick controller and real-time telemetry over WebSockets.

**Primary use case:** Indoor / micro brushed-quad where weight and simplicity matter more than GPS or advanced sensors. The operator connects a phone browser to the drone's AP and flies via virtual joysticks.

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

No external package manager, no FreeRTOS task spawning, no ESP-IDF direct calls — everything runs cooperatively inside the Arduino `loop()`.

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
└── LICENSE              # MIT License (empty file)
```

---

## 4. Core Architecture

### Single-Thread Cooperative Loop (No RTOS Tasks)

Everything runs in one Arduino `loop()` at a fixed **250 Hz** (4 ms period). The loop busy-waits using `yield()` (critical on the single-core ESP32-C3 to avoid starving the WiFi stack).

```
┌──────────────────────────────────────────────────────┐
│  setup()                                             │
│  1. Serial.begin(115200)                             │
│  2. LED init                                         │
│  3. IMU.begin() + IMU.calibrate()  ← blocks ~1s     │
│  4. Motors.begin() + Motors.stop()                   │
│  5. PID.initPIDs()                                   │
│  6. WiFi AP + HTTP + WebSocket start                 │
│  7. Timing init                                      │
└──────────────────────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│  loop() @ 250 Hz                                     │
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
| `begin()` | I2C init, MPU6050 reset, config DLPF (44 Hz), gyro ±500°/s, accel ±4g, 1 kHz sample rate. Accepts any WHO_AM_I response (clone compatibility). |
| `calibrate()` | Collects `CALIBRATION_SAMPLES` (2000) readings while stationary. Computes mean gyro offsets and accel offsets (removes 1g from Z). Initializes roll/pitch from accel. |
| `update(dt)` | Reads raw, subtracts offsets, computes accel angles, applies complementary filter: `α(gyro_integral) + (1-α)(accel_angle)`. Yaw is gyro-only (no mag). |
| `getData()` | Returns `IMUData` struct by value. |

**Important constants (from `config.h`):**
- `COMPLEMENTARY_ALPHA = 0.98` — high trust in gyro (smooth but may drift slowly)
- `GYRO_SENSITIVITY = 65.5` LSB/(°/s), `ACCEL_SENSITIVITY = 8192.0` LSB/g
- DLPF bandwidth ~44 Hz (`CONFIG register = 0x03`)

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
- D-term: `derivative = -(measurement - prevMeasurement) / dt` (note the negation — equivalent to D on measurement, not error)
- Integral is clamped to `PID_INTEGRAL_MAX` (50.0) to prevent windup
- Output clamped to configured `_minOut` / `_maxOut`
- First-run guard prevents a spike on the very first D computation

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

**Dynamic scaling:** If any motor exceeds `MOTOR_MAX`, all motors are shifted down by the overflow amount (preserves PID correction ratios). If throttle > `MOTOR_IDLE` (20), a minimum floor of 20 is enforced to keep motors spinning.

**Dependencies:** `config.h` (pin definitions, PWM constants)

### 5.4 Battery (`battery.h` / `battery.cpp`)

**Purpose:** Read 1S LiPo voltage via ADC through a voltage divider, apply a 10-sample moving average, and set warning/critical flags.

**Status: DISABLED in main sketch.** The `#include "battery.h"` and `Battery battery` object are commented out in `drone_firmware.ino` because no voltage divider is physically connected. The class is fully implemented but unused.

**Key constants:**
- `BATTERY_DIVIDER = 2.0`, `BATTERY_FULL = 4.20V`, `BATTERY_EMPTY = 3.00V`
- `BATTERY_WARNING = 3.30V`, `BATTERY_CRITICAL = 3.00V`
- 12-bit ADC, 11 dB attenuation → ~0–2.5 V reference range

### 5.5 WiFi Control (`wifi_control.h` / `wifi_control.cpp`)

**Purpose:** Manage WiFi Access Point, serve the web controller page over HTTP, handle WebSocket connections for bidirectional command/telemetry.

**Key functions:**
| Function | What it does |
|---|---|
| `begin()` | Starts AP (`YourWingC3` / `drone1234`, ch 6, 19.5 dBm TX), disables modem sleep, starts HTTP on port 80, WebSocket on port 81. |
| `update()` | Calls `httpServer.handleClient()` and `wsServer.loop()` — must be called every main loop iteration. |
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

**Important static state:** `_cmd`, `_connected`, `_lastCmdTime` are all static class members (shared across instances, singleton pattern).

**Dependencies:** `WiFi.h`, `WebServer.h`, `WebSocketsServer.h`, `config.h`, `web_page.h`

### 5.6 Web Page (`web_page.h`)

**Purpose:** A single-file HTML/CSS/JS web application stored as a `PROGMEM` string. Served at `http://192.168.4.1/`.

**Features:**
- Dual virtual joysticks (left = throttle/yaw, right = roll/pitch) with touch + mouse support
- Left stick Y (throttle) does **not** auto-center on release; X (yaw) does
- Right stick auto-centers both axes on release
- ARM / DISARM / STOP buttons
- ANGLE / RATE mode toggle
- Live telemetry display: roll, pitch, yaw angles + battery percentage
- Motor output bar indicators (FL, FR, RL, RR)
- Connection status LED dot
- Command send rate: **20 Hz** (`setInterval` every 50 ms)

**CSS:** Dark theme with Orbitron font, glassmorphism panels, accent colors.

---

## 6. Data Flow

```
MPU6050 (I2C @ 100kHz)
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

| Function | GPIO | Peripheral | Notes |
|---|---|---|---|
| Motor FR (M1, CCW) | **GPIO 0** | LEDC PWM | 20 kHz, 8-bit |
| Motor FL (M3, CW) | **GPIO 1** | LEDC PWM | 20 kHz, 8-bit |
| Motor RL (M2, CCW) | **GPIO 3** | LEDC PWM | 20 kHz, 8-bit |
| Motor RR (M4, CW) | **GPIO 4** | LEDC PWM | 20 kHz, 8-bit |
| Battery ADC | **GPIO 6** | ADC (12-bit) | Voltage divider input (disabled) |
| Onboard LED | **GPIO 8** | Digital OUT | Active LOW |
| I2C SDA (MPU6050) | **GPIO 8** | I2C (Wire) | Default ESP32-C3 I2C SDA |
| I2C SCL (MPU6050) | **GPIO 9** | I2C (Wire) | Default ESP32-C3 I2C SCL |
| WiFi AP | internal | 2.4 GHz radio | SSID: YourWingC3, ch 6 |
| USB Serial | internal | USB CDC | 115200 baud |

> **Note:** GPIO 8 is listed as both LED and SDA in the README pin table. In `config.h`, `PIN_LED = 8` and `PIN_MOTOR_FL` is reassigned to GPIO 1 (not GPIO 3 as the header comment says — the actual `#define` is `PIN_MOTOR_FL 1`). The README's pin table is outdated and contains the GPIO 8 / SDA conflict. **Trust `config.h` as the source of truth.**

**I2C configuration:** 100 kHz (default), not 400 kHz — chosen for MPU6050 clone compatibility.

---

## 8. Configuration & Tuning Parameters

All constants live in **`config.h`**. No EEPROM / persistent storage — values reset on every boot. Runtime tuning is via the Serial CLI.

### PID Gains (Angle Loop — Outer)
| Parameter | Constant | Default | Description |
|---|---|---|---|
| Roll Angle Kp | `ROLL_ANGLE_KP` | 2.0 | Proportional gain for roll angle |
| Roll Angle Ki | `ROLL_ANGLE_KI` | 0.5 | Integral — corrects steady-state drift |
| Roll Angle Kd | `ROLL_ANGLE_KD` | 0.1 | Derivative — dampens roll oscillation |
| Pitch Angle Kp | `PITCH_ANGLE_KP` | 2.0 | (Same as roll) |
| Pitch Angle Ki | `PITCH_ANGLE_KI` | 0.5 | |
| Pitch Angle Kd | `PITCH_ANGLE_KD` | 0.1 | |

### PID Gains (Rate Loop — Inner)
| Parameter | Constant | Default | Description |
|---|---|---|---|
| Roll Rate Kp | `ROLL_RATE_KP` | 0.5 | |
| Roll Rate Ki | `ROLL_RATE_KI` | 0.1 | Reduced from 0.4 (was causing windup) |
| Roll Rate Kd | `ROLL_RATE_KD` | 0.03 | |
| Pitch Rate Kp | `PITCH_RATE_KP` | 0.5 | |
| Pitch Rate Ki | `PITCH_RATE_KI` | 0.1 | |
| Pitch Rate Kd | `PITCH_RATE_KD` | 0.03 | |
| Yaw Rate Kp | `YAW_RATE_KP` | 0.8 | |
| Yaw Rate Ki | `YAW_RATE_KI` | 0.15 | |
| Yaw Rate Kd | `YAW_RATE_KD` | 0.0 | Pure PI for yaw |

### PID Limits
| Constant | Default | Description |
|---|---|---|
| `PID_MAX_OUTPUT` | 120.0 | Max motor correction (was 200, too aggressive) |
| `PID_INTEGRAL_MAX` | 50.0 | Anti-windup integral clamp |
| `MAX_ANGLE` | 45.0° | Maximum tilt angle in angle mode |
| `MAX_YAW_RATE` | 180.0°/s | Maximum yaw rotation speed |

### Control Loop
| Constant | Default | Description |
|---|---|---|
| `LOOP_FREQUENCY` | 250 Hz | Target loop rate |
| `LOOP_TIME_US` | 4000 µs | 1e6 / 250 |

### Motor Settings
| Constant | Default | Description |
|---|---|---|
| `PWM_FREQUENCY` | 20000 Hz | Above audible range |
| `PWM_RESOLUTION` | 8 | 0–255 range |
| `MOTOR_MAX` | 255 | |
| `MOTOR_MIN` | 0 | |
| `MOTOR_IDLE` | 20 | Minimum spin when armed |
| `THROTTLE_HEADROOM` | 80 | Reserved for PID corrections at full throttle |

### IMU Settings
| Constant | Default | Description |
|---|---|---|
| `MPU6050_ADDR` | 0x68 | I2C address |
| `GYRO_SENSITIVITY` | 65.5 | LSB/(°/s) at ±500°/s |
| `ACCEL_SENSITIVITY` | 8192.0 | LSB/g at ±4g |
| `COMPLEMENTARY_ALPHA` | 0.98 | Gyro trust factor |
| `CALIBRATION_SAMPLES` | 2000 | Samples for offset averaging |

### WiFi
| Constant | Default | Description |
|---|---|---|
| `WIFI_SSID` | "YourWingC3" | AP name |
| `WIFI_PASSWORD` | "drone1234" | AP password |
| `WIFI_CHANNEL` | 6 | |
| `WS_PORT` | 81 | WebSocket port |
| `HTTP_PORT` | 80 | Web server port |

### Safety
| Constant | Default | Description |
|---|---|---|
| `SIGNAL_TIMEOUT` | 500 ms | Failsafe disarm timeout |
| `TELEMETRY_INTERVAL` | 200 ms | Telemetry broadcast rate |

### Battery (unused)
| Constant | Default | Description |
|---|---|---|
| `BATTERY_DIVIDER` | 2.0 | Voltage divider ratio |
| `BATTERY_FULL` | 4.20 V | 1S LiPo full |
| `BATTERY_EMPTY` | 3.00 V | 1S LiPo empty |
| `BATTERY_WARNING` | 3.30 V | Warning threshold |
| `BATTERY_CRITICAL` | 3.00 V | Auto-disarm threshold |

### Serial CLI Tuning Commands
Format: `<Axis><Param>:<Value>` — e.g. `RP:0.7`, `AI:0.2`
- `RP/RI/RD` — Roll rate Kp/Ki/Kd
- `PP/PI/PD` — Pitch rate Kp/Ki/Kd
- `YP/YI/YD` — Yaw rate Kp/Ki/Kd
- `AP/AI/AD` — Roll angle Kp/Ki/Kd
- `BP/BI/BD` — Pitch angle Kp/Ki/Kd
- `MT:<0-3>` — Test motor at speed 80
- `MS` — Emergency stop all motors
- `ST` — Print full status (angles, rates, PID values, motor outputs)

---

## 9. Known Issues / TODOs / Incomplete Features

### Commented-Out / Disabled Code
1. **Battery monitoring is fully disabled.** The `#include "battery.h"`, `Battery battery` object, and all battery-related timing variables are commented out in `drone_firmware.ino` (lines 23, 29, 42, 152). Reason: no voltage divider connected. The battery class code exists but is dead.

2. **Battery warning LED path is unreachable** (`drone_firmware.ino:270`): `else if (false)` — this branch can never execute.

### Documentation vs Code Mismatch
3. **README pin table is outdated.** The README lists GPIO 8 as both I2C SDA and LED. In `config.h`, `PIN_LED = 8` and `PIN_MOTOR_FL = 1` (not GPIO 3 as the `.ino` header comment at line 14 states). The actual motor pin assignments in `config.h` are:
   - `PIN_MOTOR_FR = 0`, `PIN_MOTOR_RL = 3`, `PIN_MOTOR_FL = 1`, `PIN_MOTOR_RR = 4`
   
   The header comment in `drone_firmware.ino` (lines 13-14) says `Motor FL → GPIO3` and `Motor RL → GPIO1`, which is **swapped** compared to the actual `#define` values in `config.h`. **`config.h` is authoritative.**

### Design Limitations
4. **No EEPROM / persistent storage.** All PID gains and config reset on reboot. Serial tuning is ephemeral.

5. **Yaw drift.** Yaw is gyro-only integrated (no magnetometer), so it drifts continuously. This is expected for a 6-DOF IMU but is a known limitation.

6. **Single-threaded WiFi + flight control.** The `yield()` in the loop-wait is a workaround for ESP32-C3's single core. Under heavy WiFi load, loop timing could jitter.

7. **Web page loads Google Fonts from CDN** (`web_page.h:15`). If the phone has no internet while connected to the drone AP, fonts will fail to load (graceful degradation, but the UI looks worse).

8. **No DShot/ESC protocol.** Motors are driven with simple analog PWM through MOSFETs (brushed DC only). Cannot be used with brushless ESCs.

9. **Telemetry sends battery as 0.0/0** since battery monitoring is disabled (`drone_firmware.ino:253`).

10. **LICENSE file is empty** — contains no text.

---

## 10. Entry Points

### `setup()` — `drone_firmware.ino:92`
Initialization order:
1. Serial at 115200
2. LED pin as OUTPUT
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
| **YourWingC3** | Project codename (the WiFi SSID and branding) |
| **0716 motors** | 7 mm diameter × 16 mm length coreless brushed DC motors — the specific motor type this firmware targets |
| **X-Quad** | Frame configuration where motors form an X shape (not + shape); affects mixing formulas |
| **CCW / CW** | Counter-clockwise / Clockwise motor spin direction |
| **FR / FL / RL / RR** | Front-Right, Front-Left, Rear-Left, Rear-Right motor positions |
| **M1–M4** | Motor indices: M1=FR, M2=RL, M3=FL, M4=RR |
| **Angle mode** | Self-leveling mode (flightMode=0): stick position = desired tilt angle |
| **Rate mode** | Acro/manual mode (flightMode=1): stick position = desired rotation rate |
| **Cascaded PID** | Two-layer PID: outer angle loop outputs desired rate → inner rate loop outputs motor correction |
| **D-on-measurement** | Derivative computed on measurement change (not error change) to avoid derivative kick on setpoint jumps |
| **Complementary filter** | Sensor fusion: `α × (gyro integration) + (1-α) × (accel angle)`. α=0.98 trusts gyro heavily |
| **Anti-windup** | Integral clamping to prevent the I-term from accumulating excessively during saturation |
| **Dynamic scaling** | When any motor overflows, all motors are shifted down proportionally instead of hard-clamping individual motors |
| **THROTTLE_HEADROOM** | PWM range reserved for PID corrections above commanded throttle (prevents loss of control at full throttle) |
| **MOTOR_IDLE** | Minimum PWM (20/255) applied to all motors when armed and throttle > idle — keeps motors spinning |
| **Failsafe** | Auto-disarm if no WebSocket command received for `SIGNAL_TIMEOUT` (500 ms) |
| **LEDC** | ESP32's LED Control peripheral — used here for PWM motor output (not LEDs) |
| **DLPF** | Digital Low-Pass Filter — MPU6050 hardware filter set to ~44 Hz bandwidth |
| **PROGMEM** | Arduino macro to store data in flash memory (used for the web page HTML) |
| **`yield()`** | Arduino call that feeds the RTOS idle task — essential on single-core ESP32-C3 to prevent WiFi stack starvation |
| **ST command** | Serial CLI command that prints a full status dump (angles, rates, PID values, motor outputs, loop Hz) |
| **`_firstRun`** | PID flag that prevents a derivative spike on the very first `compute()` call |
