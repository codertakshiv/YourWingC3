# YourWingC3

**Lightweight ESP32-C3 flight controller for micro brushed quadcopters, flown from your phone browser.**

---

## Overview

YourWingC3 is a complete flight controller firmware for micro quadcopters built on the **ESP32-C3 Super Mini**. Instead of a traditional radio transmitter, you fly the drone from a browser-based dual-joystick UI served over WiFi — connect your phone, open the page, and fly. It is designed for hobbyists building indoor micro brushed quads using cheap, common components (MPU6050, 0716 coreless motors, MOSFET drivers, 1S LiPo).

## Features

- **250 Hz cascaded PID control loop** — outer angle loop + inner rate loop for self-leveling stability
- **MPU6050 IMU** with complementary filter (98% gyro / 2% accelerometer) and startup calibration
- **WiFi Access Point** with a browser-based dual-joystick controller page (no app install needed)
- **Live telemetry** streamed over WebSocket at 5 Hz (roll, pitch, yaw, motor outputs, loop rate)
- **Serial CLI** for live PID tuning without reflashing
- **Failsafe auto-disarm** if no command received for 500 ms
- **X-quad motor mixing** with dynamic overflow scaling and throttle headroom for PID corrections
- **ANGLE mode** (self-leveling) and **RATE mode** (acro/manual) switchable from the UI

## Hardware Requirements

| Component | Notes |
|---|---|
| ESP32-C3 Super Mini | The microcontroller. Single-core RISC-V, 160 MHz |
| MPU6050 | 6-axis IMU (accel + gyro), I2C interface, address 0x68 |
| 4x 0716 coreless motors | 7 mm diameter, brushed DC. M1 Front-Right (CCW), M2 Rear-Left (CCW), M3 Front-Left (CW), M4 Rear-Right (CW) |
| 4x N-Channel MOSFETs | Logic-level gate (e.g. SI2302), one per motor, with flyback diodes |
| 1S (3.7 V) LiPo battery | Powers the entire system |
| Quadcopter frame | Any micro X-config frame that fits 0716 motors |

> **Battery monitoring:** The firmware includes ADC-based battery voltage reading code, but it is currently **disabled** because no voltage divider is wired by default. You will not get low-battery warnings out of the box. To enable it, wire a voltage divider to GPIO 6 and uncomment the battery-related lines in `drone_firmware.ino`.

## Wiring / Pin Table

All pin assignments are defined in `config.h`:

| Function | GPIO | Peripheral |
|---|---|---|
| Motor FR (M1, CCW) | **GPIO 0** | LEDC PWM, 20 kHz |
| Motor FL (M3, CW) | **GPIO 1** | LEDC PWM, 20 kHz |
| Motor RL (M2, CCW) | **GPIO 3** | LEDC PWM, 20 kHz |
| Motor RR (M4, CW) | **GPIO 4** | LEDC PWM, 20 kHz |
| Battery ADC | **GPIO 6** | ADC (12-bit, currently unused) |
| Status LED | **GPIO 8** | Digital output, active LOW |
| I2C SDA (MPU6050) | **GPIO 8** | I2C data (default Wire) |
| I2C SCL (MPU6050) | **GPIO 9** | I2C clock (default Wire) |

> **Note:** GPIO 8 is shared between the onboard LED and I2C SDA. This works because the LED is driven as a simple digital output while I2C SDA is an open-drain line — the LED LOW/HIGH toggle does not conflict with I2C communication on this pin.

## Getting Started

### Prerequisites

1. **Arduino IDE** installed
2. **ESP32 Board Package** — In Arduino IDE, go to *File > Preferences > Additional Board Manager URLs* and add the Espressif URL, then install `esp32` by Espressif Systems from *Tools > Board > Boards Manager*
3. **WebSockets Library** — Install `WebSockets` by Links2004 from *Sketch > Include Library > Manage Libraries*

### Board Settings

In *Tools*, set:

| Setting | Value |
|---|---|
| Board | ESP32C3 Dev Module |
| USB CDC On Boot | Enabled |
| Flash Mode | QIO or DIO |
| Upload Speed | 921600 |

### Flashing

1. Connect the ESP32-C3 via USB
2. Open `drone_firmware.ino` in the Arduino IDE
3. Select the correct COM port under *Tools > Port*
4. Click **Upload**

## First Boot & Calibration

1. **Keep the drone perfectly still** on a flat surface during the first ~1 second after power-on. The firmware collects 2000 IMU samples to calibrate gyro and accelerometer offsets. Any movement during this period will cause angle drift.

2. Once calibration completes, the status LED blinks 3 times and the Serial Monitor prints the WiFi credentials.

3. **Connect to the WiFi network:**
   - SSID: `YourWingC3`
   - Password: `drone1234`

4. **Open a browser** and navigate to `http://192.168.4.1`. You will see the dual-joystick controller page with live telemetry.

5. **To fly:** Lower the throttle to zero, tap **ARM**, then raise the throttle and use the right stick for roll/pitch.

## Serial CLI Commands

Connect via USB Serial at **115200 baud**. Send any of the following commands (terminated with newline):

| Command | Action |
|---|---|
| `RP:<value>` | Set Roll Rate Kp |
| `RI:<value>` | Set Roll Rate Ki |
| `RD:<value>` | Set Roll Rate Kd |
| `PP:<value>` | Set Pitch Rate Kp |
| `PI:<value>` | Set Pitch Rate Ki |
| `PD:<value>` | Set Pitch Rate Kd |
| `YP:<value>` | Set Yaw Rate Kp |
| `YI:<value>` | Set Yaw Rate Ki |
| `YD:<value>` | Set Yaw Rate Kd |
| `AP:<value>` | Set Roll Angle Kp |
| `AI:<value>` | Set Roll Angle Ki |
| `AD:<value>` | Set Roll Angle Kd |
| `BP:<value>` | Set Pitch Angle Kp |
| `BI:<value>` | Set Pitch Angle Ki |
| `BD:<value>` | Set Pitch Angle Kd |
| `MT:<0-3>` | Test motor at speed 80 (0=FR, 1=RL, 2=FL, 3=RR) |
| `MS` | Emergency stop all motors |
| `ST` | Print full status (angles, rates, PID gains, motor outputs, loop Hz) |

**Examples:** `RP:0.7` sets Roll Rate Kp to 0.7. `AP:2.5` sets Roll Angle Kp to 2.5. `ST` prints the full status dump.

## Safety Notes

- **Failsafe:** The drone auto-disarms if no WebSocket command is received for **500 ms** (e.g. phone goes out of range or browser tab is closed).
- **Arm safety:** You cannot arm unless throttle is at zero. The ARM button in the browser enforces this.
- **PID gains reset on reboot.** There is no persistent storage (EEPROM/flash) yet — any tuning done via the Serial CLI is lost when the drone powers off.
- **Yaw drift:** Yaw is integrated from the gyroscope only (no magnetometer). It will drift slowly over time. This is normal for a 6-DOF IMU setup.
- **Always remove propellers** when testing motors on the bench or flashing new firmware.

## Known Limitations

- **Brushed motors only.** Motors are driven via analog PWM through MOSFETs. No support for brushless ESC protocols (DShot, OneShot, etc.).
- **No GPS** — position hold and return-to-home are not available.
- **No barometer** — altitude hold is not available.
- **Single-core WiFi + flight control** — the ESP32-C3 runs both the WiFi stack and the 250 Hz flight loop on one core. The firmware uses `yield()` during loop timing to prevent WiFi starvation, but jitter may occur under heavy network load.
- **Battery monitoring not wired by default** — the code exists but is disabled. You need to add a voltage divider to GPIO 6 and uncomment the battery code to use it.
- **No persistent configuration** — all PID gains and tuning parameters reset on every reboot.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
