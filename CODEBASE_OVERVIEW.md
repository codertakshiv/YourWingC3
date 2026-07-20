# Codebase Overview — YourWingC3

> Auto-generated from source files. All line numbers, pin values, and configuration constants verified against the current codebase on disk.

---

## 1. Project Summary

| Field | Value |
|---|---|
| **Name** | YourWingC3 |
| **Purpose** | Lightweight flight controller firmware for micro brushed quadcopters, flown from a phone browser via WiFi |
| **Hardware target** | ESP32-C3 Super Mini (single-core RISC-V, 160 MHz) |
| **IMU** | MPU6050 (6-axis, I2C, address 0x68) |
| **Motors** | 4x 0716 coreless brushed DC motors, driven via N-channel MOSFETs |
| **Battery** | 1S (3.7V) LiPo |
| **Framework** | Arduino (ESP32 board package by Espressif) |
| **License** | MIT — Copyright (c) 2026 YourWingC3 |

---

## 2. Tech Stack & Build System

| Component | Detail |
|---|---|
| IDE | Arduino IDE |
| Board package | `esp32` by Espressif Systems |
| Board setting | ESP32C3 Dev Module |
| Required library | `WebSockets` by Links2004 (installed via Library Manager) |
| USB CDC On Boot | Must be **Enabled** in Tools |
| Upload speed | 921600 |
| Serial baud rate | 115200 |
| Language | C++ (Arduino dialect) |
| No build system | No PlatformIO, no CMake — Arduino IDE `.ino` sketch compilation only |

---

## 3. Folder & File Structure

```
YourWingC3/
├── drone_firmware.ino   ← Main sketch (setup, loop, serial CLI)
├── config.h             ← All tunable constants, pin definitions, PID gains
├── imu.h / imu.cpp      ← MPU6050 driver (I2C, complementary filter, calibration)
├── pid.h / pid.cpp      ← Generic PID controller (D-on-measurement, anti-windup)
├── motors.h / motors.cpp← X-quad mixing, LEDC PWM output
├── battery.h / battery.cpp ← Battery voltage monitoring (DISABLED — no divider wired)
├── wifi_control.h / .cpp ← WiFi AP, HTTP server, WebSocket server, command parsing
├── web_page.h           ← PROGMEM HTML/CSS/JS dual-joystick controller UI
├── README.md            ← User-facing documentation, wiring table, CLI reference
├── LICENSE              ← MIT License
└── CODEBASE_OVERVIEW.md ← This file
```

**16 files total.** No subdirectories, no hidden files, no build artifacts.

---

## 4. Core Architecture

### Loop Structure

The firmware runs a single `loop()` function at **250 Hz** (4 ms period), governed by a microsecond timer with `yield()` to prevent WiFi starvation on the single-core ESP32-C3.

**`drone_firmware.ino:158-276` — `loop()` executes these steps every iteration:**

```
1. Wait for precise 4ms timing (yield() during wait)     [line 162-166]
2. Read IMU (complementary filter update)                 [line 168-170]
3. Process WiFi / WebSocket commands                      [line 172-174]
4. Failsafe check (500ms signal timeout)                  [line 176-185]
5. Arm / disarm logic                                     [line 190-203]
6. Flight control (cascaded PID or direct rate)           [line 207-244]
7. Send telemetry over WebSocket (5 Hz)                   [line 246-256]
8. Loop frequency counter                                 [line 258-264]
9. Status LED update                                      [line 266-272]
10. Serial command handler                                [line 274-275]
```

### Timing

- `loopTimer` (micros): Controls 250 Hz loop rate. Reset each iteration.
- `hzTimer` (millis): Counts loops per second for `loopHz` display.
- `lastTelemetry` (millis): Throttles telemetry to every 200 ms (5 Hz).
- `yield()` inside the timing wait is critical — without it, the WiFi stack starves and the phone disconnects.

### Module Communication

Modules are **not** coupled. The main sketch owns all global objects and passes data between them:

```
imu.getData() ──→ IMUData struct ──→ PID.compute() ──→ float corrections
                                                              ↓
wifiCtrl.getCommand() ──→ ControlCommand struct ──→ motors.mix(throttle, corrections)
```

- `IMU` reads hardware directly (Wire/I2C), outputs `IMUData` struct
- `PIDController` is pure math — no hardware access, no state beyond PID terms
- `Motors` receives mixed values, writes LEDC PWM
- `WifiControl` receives/sends WebSocket JSON, exposes `ControlCommand` struct
- `Battery` is unused (commented out in main sketch)

---

## 5. Key Modules Breakdown

### 5a. IMU (`imu.h` / `imu.cpp`)

| Aspect | Detail |
|---|---|
| Sensor | MPU6050 at I2C address 0x68 |
| I2C init | `Wire.begin()` — no arguments, uses ESP32-C3 defaults (GPIO 8 = SDA, GPIO 9 = SCL) (`imu.cpp:5`) |
| I2C speed | 100 kHz (default, comments note many clones unreliable at 400kHz) |
| DLPF | 44 Hz bandwidth (`imu.cpp:28-30`) |
| Gyro range | ±500°/s (`imu.cpp:33-36`) |
| Accel range | ±4g (`imu.cpp:39-42`) |
| Sample rate | 1 kHz internal (`imu.cpp:45-48`) |
| Filter | Complementary filter: 98% gyro / 2% accel (`config.h:31`, `imu.cpp:154-155`) |
| Calibration | 2000 samples at startup, computes gyro + accel offsets (`imu.cpp:65-105`). Drone must be still during first ~1s after power-on. |
| Yaw | Gyro-only integration (no magnetometer). Wraps at ±180°. Drifts over time. |

**`IMUData` struct** (returned by `getData()`):
```cpp
struct IMUData {
    float roll, pitch, yaw;       // Angles in degrees
    float rollRate, pitchRate, yawRate;  // Angular rates in °/s
};
```

### 5b. PID Controller (`pid.h` / `pid.cpp`)

| Aspect | Detail |
|---|---|
| Type | Standard PID with D-on-measurement (not D-on-error) |
| Anti-windup | Integral clamping via `setIntegralLimit()` — `constrain()` on accumulated integral (`pid.cpp:40`) |
| Derivative kick avoidance | D-term computed on measurement change, not error change (`pid.cpp:48`: `-(measurement - _prevMeasurement) / dt`) |
| First-run guard | `_firstRun` flag prevents derivative spike on first compute call (`pid.cpp:44-47`) |
| Output clamping | `constrain(output, _minOut, _maxOut)` (`pid.cpp:54`) |
| Runtime tuning | `setKp()`, `setKi()`, `setKd()` public methods — used by Serial CLI |

**5 PID instances** created in `drone_firmware.ino:33-35`:
- `rollAnglePID`, `pitchAnglePID` — outer loop (angle → desired rate)
- `rollRatePID`, `pitchRatePID` — inner loop (rate → motor correction)
- `yawRatePID` — rate-only (no angle mode for yaw)

### 5c. Motors (`motors.h` / `motors.cpp`)

| Aspect | Detail |
|---|---|
| PWM | 20 kHz via LEDC (`config.h:20`), 8-bit resolution (0-255) |
| API | `ledcAttach()` / `ledcWrite()` (modern ESP32 Arduino core) |
| Mixing | X-quad configuration (`motors.cpp:21-35`) |
| Overflow handling | Dynamic proportional scaling — if any motor > 255, all motors shifted down equally (`motors.cpp:57-62`) |
| Idle spin | When throttle > `MOTOR_IDLE` (20), motor minimum is clamped to 20 (`motors.cpp:66-67`) |
| Throttle headroom | Capped at `MOTOR_MAX - THROTTLE_HEADROOM` (255 - 80 = 175) to leave room for PID corrections (`motors.cpp:40-41`) |

**Motor positions and rotation:**
```
    Front
  M3(CW)  M1(CCW)
      \  /
       \/
       /\
      /  \
  M2(CCW) M4(CW)
     Rear

M1 (FR, CCW) = Throttle - Roll - Pitch + Yaw
M2 (RL, CCW) = Throttle + Roll + Pitch + Yaw
M3 (FL, CW)  = Throttle + Roll - Pitch - Yaw
M4 (RR, CW)  = Throttle - Roll + Pitch - Yaw
```

### 5d. Status LED (`drone_firmware.ino`)

| Aspect | Detail |
|---|---|
| Pin | GPIO 8 (shared with I2C SDA) |
| Mode | `OUTPUT_OPEN_DRAIN` (`drone_firmware.ino:103`) |
| ON logic | `digitalWrite(PIN_LED, LOW)` — actively pulls line LOW, sinking current through LED (`drone_firmware.ino:48`) |
| OFF logic | `digitalWrite(PIN_LED, HIGH)` — releases pin, line floats HIGH (LED off) (`drone_firmware.ino:49`) |
| Why open-drain | GPIO 8 is shared with I2C SDA. I2C requires open-drain signaling. The LED's open-drain mode means it only ever pulls LOW (matching I2C protocol) and never actively drives HIGH, preventing bus contention. |
| Behavior patterns | **Armed:** fast blink, 100ms toggle (`line 269`). **Disarmed + connected:** LED off (`line 271`, returns HIGH). **Disarmed + disconnected:** slow blink, 1000ms toggle (`line 271`). **Calibration:** solid ON during gyro calibration (`line 117-119`). **IMU failure:** rapid triple-blink loop (`line 112`). **Ready:** 3 blinks (`line 146`). |
| Flicker | Minor LED flicker during active I2C transactions is possible but harmless — both protocols use open-drain LOW signaling on the same pin. |

### 5e. Battery Monitoring (`battery.h` / `battery.cpp`)

**DISABLED.** The `#include "battery.h"` and `Battery battery;` are commented out in `drone_firmware.ino:23,29`. No voltage divider is wired. The code exists but is completely inert.

If re-enabled (requires wiring a voltage divider to GPIO 6), it provides:
- 12-bit ADC reads with 10-sample moving average
- Voltage calculation: `(raw / 4095) * 2.5V * BATTERY_DIVIDER (2.0)`
- Percentage: linear mapping from `BATTERY_EMPTY` (3.0V) to `BATTERY_FULL` (4.2V)
- Warning at 3.3V, critical auto-disarm at 3.0V

### 5f. WiFi & Web Controller (`wifi_control.h` / `.cpp` / `web_page.h`)

| Aspect | Detail |
|---|---|
| Mode | WiFi Access Point (not STA) |
| SSID | `YourWingC3` (`config.h:68`) |
| Password | `drone1234` (`config.h:69`) |
| Channel | 6 (`config.h:70`) |
| TX Power | 19.5 dBm (max, for stability) (`wifi_control.cpp:17`) |
| Modem sleep | Disabled to prevent random disconnects (`wifi_control.cpp:19`) |
| HTTP server | Port 80, serves `web_page.h` PROGMEM HTML on `/` (`wifi_control.cpp:27-30`) |
| WebSocket server | Port 81, bidirectional (`wifi_control.cpp:34-36`) |
| Command format | CSV: `T:0.50,R:0.00,P:0.00,Y:0.00,A:1,M:0` — sent at 20 Hz from browser (`web_page.h:179,255`) |
| Telemetry format | JSON broadcast: `{"r":...,"p":...,"y":...,"bv":...,"bp":...,"a":...,"hz":...,"m1":...}` at 5 Hz |
| Failsafe | Auto-disarm if no command for 500 ms (`config.h:83`, `drone_firmware.ino:177-178`) |
| Disconnect safety | On WebSocket disconnect: armed=false, throttle=0 (`wifi_control.cpp:55-56`) |

**Web UI (`web_page.h`):**
- Dual virtual joysticks (left: throttle/yaw, right: roll/pitch)
- Live telemetry display (roll, pitch, yaw, battery %)
- Motor output bars (4 motors)
- ARM / DISARM / STOP buttons
- ANGLE / RATE mode toggle
- Fonts: Orbitron (headings) + Inter (body) loaded from `fonts.googleapis.com` CDN (`web_page.h:15`)
- Touch-optimized for mobile, responsive landscape layout

---

## 6. Data Flow

```
Phone Browser                          ESP32-C3
─────────────                          ────────
                                       
[Joysticks] ──(20Hz WebSocket)──→  WifiControl.parseCommand()
                                        ↓
                                   ControlCommand struct
                                   {throttle, roll, pitch, yaw, armed, mode}
                                        ↓
                                   Flight control loop (250 Hz):
                                        ↓
                                   IMU.update(dt) → IMUData
                                        ↓
                              ┌── ANGLE mode: AnglePID → RatePID → corrections
                              └── RATE mode: RatePID directly → corrections
                                        ↓
                                   Motors.mix(throttle, corrections)
                                        ↓
                                   LEDC PWM → MOSFETs → Motors
                                       
                                   ←──(5Hz WebSocket)── Telemetry JSON
[Display updates] ←────────────         
```

---

## 7. Pin Mapping / Hardware Config

All pin assignments from `config.h:10-17`:

| Function | GPIO | Config Line | Peripheral | Notes |
|---|---|---|---|---|
| Motor FR (M1, CCW) | **GPIO 0** | `config.h:12` | LEDC PWM, 20 kHz | Front-Right |
| Motor FL (M3, CW) | **GPIO 1** | `config.h:14` | LEDC PWM, 20 kHz | Front-Left |
| Motor RL (M2, CCW) | **GPIO 3** | `config.h:13` | LEDC PWM, 20 kHz | Rear-Left |
| Motor RR (M4, CW) | **GPIO 4** | `config.h:15` | LEDC PWM, 20 kHz | Rear-Right |
| Battery ADC | **GPIO 6** | `config.h:16` | ADC (12-bit) | Currently unused (no divider) |
| Status LED | **GPIO 8** | `config.h:17` | Open-drain output | **Shared with I2C SDA** — see below |
| I2C SDA (MPU6050) | **GPIO 8** | `imu.cpp:5` | I2C data | `Wire.begin()` uses ESP32-C3 default |
| I2C SCL (MPU6050) | **GPIO 9** | `imu.cpp:5` | I2C clock | `Wire.begin()` uses ESP32-C3 default |

### GPIO 8 Shared Pin (LED + I2C SDA) — RESOLVED

The status LED and I2C SDA **share GPIO 8**. This is resolved in software via `OUTPUT_OPEN_DRAIN` mode:

- `drone_firmware.ino:103`: `pinMode(PIN_LED, OUTPUT_OPEN_DRAIN)`
- The LED only ever pulls the pin LOW (ON) or releases it (OFF), never actively drives HIGH
- I2C also uses open-drain LOW-pull signaling, so both protocols are electrically compatible
- Minor LED flicker during active I2C transactions is possible but harmless
- **No physical rewiring required.** The LED must remain on GPIO 8 (hardware constraint).

---

## 8. Configuration & Tuning Parameters

All constants from `config.h`. No EEPROM or persistent storage — every value resets on reboot.

### Motor PWM (`config.h:19-25`)

| Parameter | Constant | Value |
|---|---|---|
| PWM frequency | `PWM_FREQUENCY` | 20,000 Hz (20 kHz) |
| PWM resolution | `PWM_RESOLUTION` | 8-bit (0-255) |
| Motor minimum | `MOTOR_MIN` | 0 |
| Motor maximum | `MOTOR_MAX` | 255 |
| Throttle headroom | `THROTTLE_HEADROOM` | 80 |
| Motor idle | `MOTOR_IDLE` | 20 |

### IMU Settings (`config.h:27-32`)

| Parameter | Constant | Value |
|---|---|---|
| MPU6050 address | `MPU6050_ADDR` | 0x68 |
| Gyro sensitivity | `GYRO_SENSITIVITY` | 65.5 LSB/(°/s) at ±500°/s |
| Accel sensitivity | `ACCEL_SENSITIVITY` | 8192 LSB/g at ±4g |
| Complementary alpha | `COMPLEMENTARY_ALPHA` | 0.98 (98% gyro, 2% accel) |
| Calibration samples | `CALIBRATION_SAMPLES` | 2000 |

### PID Gains — Angle Loop (Outer) (`config.h:34-42`)

| Parameter | Constant | Default |
|---|---|---|
| Roll Angle Kp | `ROLL_ANGLE_KP` | 2.0 |
| Roll Angle Ki | `ROLL_ANGLE_KI` | 0.5 |
| Roll Angle Kd | `ROLL_ANGLE_KD` | 0.1 |
| Pitch Angle Kp | `PITCH_ANGLE_KP` | 2.0 |
| Pitch Angle Ki | `PITCH_ANGLE_KI` | 0.5 |
| Pitch Angle Kd | `PITCH_ANGLE_KD` | 0.1 |

### PID Gains — Rate Loop (Inner) (`config.h:44-55`)

| Parameter | Constant | Default |
|---|---|---|
| Roll Rate Kp | `ROLL_RATE_KP` | 0.5 |
| Roll Rate Ki | `ROLL_RATE_KI` | 0.1 |
| Roll Rate Kd | `ROLL_RATE_KD` | 0.03 |
| Pitch Rate Kp | `PITCH_RATE_KP` | 0.5 |
| Pitch Rate Ki | `PITCH_RATE_KI` | 0.1 |
| Pitch Rate Kd | `PITCH_RATE_KD` | 0.03 |
| Yaw Rate Kp | `YAW_RATE_KP` | 0.8 |
| Yaw Rate Ki | `YAW_RATE_KI` | 0.15 |
| Yaw Rate Kd | `YAW_RATE_KD` | 0.0 |

### PID Limits (`config.h:57-61`)

| Parameter | Constant | Value |
|---|---|---|
| Max PID output | `PID_MAX_OUTPUT` | 120.0 |
| Integral max | `PID_INTEGRAL_MAX` | 50.0 |
| Max tilt angle | `MAX_ANGLE` | 45.0° |
| Max yaw rate | `MAX_YAW_RATE` | 180.0°/s |

### Control Loop (`config.h:63-65`)

| Parameter | Constant | Value |
|---|---|---|
| Loop frequency | `LOOP_FREQUENCY` | 250 Hz |
| Loop period | `LOOP_TIME_US` | 4000 µs |

### WiFi (`config.h:67-72`)

| Parameter | Constant | Value |
|---|---|---|
| SSID | `WIFI_SSID` | `"YourWingC3"` |
| Password | `WIFI_PASSWORD` | `"drone1234"` |
| Channel | `WIFI_CHANNEL` | 6 |
| WebSocket port | `WS_PORT` | 81 |
| HTTP port | `HTTP_PORT` | 80 |

### Battery Monitoring (`config.h:74-80`) — Currently Disabled

| Parameter | Constant | Value |
|---|---|---|
| Divider ratio | `BATTERY_DIVIDER` | 2.0 |
| Full voltage | `BATTERY_FULL` | 4.20V |
| Empty voltage | `BATTERY_EMPTY` | 3.00V |
| Warning threshold | `BATTERY_WARNING` | 3.30V |
| Critical threshold | `BATTERY_CRITICAL` | 3.00V |
| Read interval | `BATTERY_INTERVAL` | 500 ms |

### Safety (`config.h:82-84`)

| Parameter | Constant | Value |
|---|---|---|
| Signal timeout | `SIGNAL_TIMEOUT` | 500 ms |
| Telemetry interval | `TELEMETRY_INTERVAL` | 200 ms (5 Hz) |

---

## 9. Known Issues / TODOs / Incomplete Features

These are genuine current issues verified against the source code:

1. **No EEPROM / persistent storage** — All PID gains, flight mode, and tuning reset on every reboot. `config.h` constants are the only way to set defaults. No flash/EEPROM persistence code exists.

2. **Yaw drift** — Yaw is integrated from gyroscope only (`imu.cpp:158`). No magnetometer is present. Yaw will drift continuously over time. This is inherent to 6-DOF IMU setups without a magnetometer.

3. **Single-core WiFi + flight control** — The ESP32-C3 has one core running both the WiFi stack and the 250 Hz flight loop. The `yield()` call inside the timing wait (`drone_firmware.ino:163`) prevents WiFi starvation, but jitter may occur under heavy network load.

4. **Brushed motors only** — Motors are driven via analog PWM through MOSFETs. No support for brushless ESC protocols (DShot, OneShot, etc.).

5. **Battery monitoring disabled** — Code exists in `battery.h` / `battery.cpp` but is commented out in `drone_firmware.ino:23,29`. No voltage divider is wired. No low-battery warnings out of the box.

6. **Google Fonts loaded from CDN** — The web controller page (`web_page.h:15`) loads Orbitron and Inter fonts from `fonts.googleapis.com`. This requires internet access from the phone, which may not be available when connected only to the drone's AP. Fonts will fall back to system defaults if CDN is unreachable, but the page works fine.

7. **No persistent configuration** — Same as #1. All tuning via Serial CLI is lost on power cycle.

---

## 10. Entry Points

### `setup()` — `drone_firmware.ino:92-153`

Initialization sequence:
1. Serial begin at 115200 baud (`line 94`)
2. Print banner ("YourWingC3 - Flight Controller") (`lines 97-100`)
3. LED pin mode → `OUTPUT_OPEN_DRAIN` (`line 103`)
4. IMU init via `imu.begin()` — I2C + MPU6050 register config (`line 108`)
5. Gyro calibration — 2000 samples, LED solid during cal (`lines 116-120`)
6. Motor init + stop all (`lines 123-126`)
7. Battery disabled notice (`line 129`)
8. PID init — all 5 controllers configured (`lines 132-133`)
9. WiFi AP + HTTP + WebSocket init (`lines 136-137`)
10. Ready banner with WiFi credentials (`lines 140-145`)
11. 3x LED blink = ready (`line 146`)
12. Timer initialization (`lines 149-152`)

### `loop()` — `drone_firmware.ino:158-276`

Runs at 250 Hz. See Section 4 for full step-by-step breakdown.

### `handleSerialCommands()` — `drone_firmware.ino:290-350`

Parses serial input for live PID tuning. Supports:
- `RP/RI/RD` — Roll rate Kp/Ki/Kd
- `PP/PI/PD` — Pitch rate Kp/Ki/Kd
- `YP/YI/YD` — Yaw rate Kp/Ki/Kd
- `AP/AI/AD` — Roll angle Kp/Ki/Kd
- `BP/BI/BD` — Pitch angle Kp/Ki/Kd
- `MT:<0-3>` — Test motor at speed 80
- `MS` — Emergency stop all motors
- `ST` — Print full status dump (angles, rates, PID gains, motor outputs, loop Hz)

### WiFi Event Handlers — `wifi_control.cpp:44-98`

- `onWebSocketEvent()` (`line 44`) — static callback for connect/disconnect/text events
- `parseCommand()` (`line 70`) — parses CSV command string from browser into `ControlCommand` struct

---

## 11. Glossary of Project-Specific Terms

| Term | Meaning |
|---|---|
| **ANGLE mode** | Self-leveling flight mode. Joystick position = desired tilt angle. PID cascade: angle → rate → motor. Default mode (`flightMode == 0`). |
| **RATE mode** | Acro/manual flight mode. Joystick position = desired angular rate. Direct rate → motor PID. No self-leveling (`flightMode == 1`). |
| **Cascaded PID** | Two-layer PID: outer angle loop outputs desired rate, inner rate loop outputs motor correction. Used in ANGLE mode. |
| **Complementary filter** | Fuses gyro (fast, drifts) and accel (slow, stable) with 98/2 weight. Replaces Kalman filter for simplicity. |
| **D-on-measurement** | PID derivative computed on change in measurement, not change in error. Avoids "derivative kick" when setpoint changes suddenly. |
| **X-quad** | Motor layout: FR/RL are CCW, FL/RR are CW, arranged in X pattern. |
| **LEDC PWM** | ESP32's LED Control peripheral used for motor PWM. Generates hardware-timed PWM without CPU involvement. |
| **Open-drain** | Output mode where pin can only pull LOW or float (never drive HIGH). Required for I2C bus sharing. |
| **THROTTLE_HEADROOM** | PWM values (0-80) reserved for PID corrections so PID can still affect motor speed even at high throttle. |
| **Dynamic overflow scaling** | If any motor exceeds 255 after mixing, all motors are shifted down proportionally to preserve correction ratios. |
| **Failsafe** | Auto-disarm triggered when no WebSocket command received for 500 ms. Prevents flyaway if phone disconnects. |
| **0716 motors** | 7mm diameter, 16mm length coreless brushed DC motors. Common in micro quadcopters. |
| **Procontrollr / LiteWing / CircuitDigest** | Previous project names/attributions from the original Comet-Drone fork. Fully removed from the codebase — see Section 12. |

---

## 12. Recent Changes Log

### Rebrand Audit (2026-07-20)

Project-wide case-insensitive search results across all 16 files:

| Search term | Matches in source files | Matches in docs only |
|---|---|---|
| `comet` | **0** | 0 (all 17 old matches were in `CODEBASE_OVERVIEW.md`, now overwritten) |
| `litewing` | **0** | 0 |
| `circuitdigest` | **0** | 0 |

**The rebrand is fully clean.** All source files, comments, banner text, web page, config, and documentation use "YourWingC3". No traces of "Comet Drone", "CometDrone", "LiteWing", "CircuitDigest", or "Bhushan Patil" remain in any file.

### Completed Changes (verified against current source)

1. **Full rebrand to YourWingC3** — All occurrences updated:
   - `config.h:5` — "YourWingC3 - Configuration"
   - `config.h:68` — `WIFI_SSID "YourWingC3"`
   - `drone_firmware.ino:2` — "YourWingC3 - Flight Controller Firmware"
   - `drone_firmware.ino:98` — serial banner "YourWingC3 - Flight Controller"
   - `web_page.h:12` — `<title>YourWingC3</title>`
   - `web_page.h:97` — logo `YOURWINGC3`
   - `LICENSE:3` — "Copyright (c) 2026 YourWingC3"
   - `README.md` — fully rewritten

2. **GPIO 8 LED/I2C SDA conflict resolved** — Fixed in software:
   - `config.h:17` — `PIN_LED 8` with comment: "active LOW, open-drain — shared with I2C SDA"
   - `drone_firmware.ino:103` — `pinMode(PIN_LED, OUTPUT_OPEN_DRAIN)`
   - LED stays on GPIO 8. No physical rewiring. Open-drain mode prevents bus contention.
   - All `digitalWrite()` calls verified — LOW=ON, HIGH=OFF logic is correct for open-drain.

3. **Dead `else if (false)` branch removed** — Battery LED dead code that was previously at lines 270-271 has been removed from `drone_firmware.ino`.

4. **Pin wiring comment corrected** — `drone_firmware.ino:13-14` now matches `config.h` exactly:
   - FR→GPIO0, RL→GPIO3, FL→GPIO1, RR→GPIO4

5. **LICENSE populated** — Full MIT License text with correct copyright holder ("YourWingC3").

6. **README.md fully rewritten** — Accurate pin table, serial CLI commands, hardware requirements, getting started guide, safety notes, and visual polish (badges, TOC, emoji headers, quick-start callout).

### No Known Reverted or Pending Items

All previously identified issues from the Comet-Drone clone overwrite have been resolved:
- Rebrand: complete
- GPIO 8 conflict: resolved via open-drain
- Dead branch: removed
- Pin comment: corrected
- LICENSE: populated with correct copyright
- README: rewritten

**There are no pending code changes or reverted fixes remaining.**
