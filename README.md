# Teensy 4.1 Flight Controller - Stabilize Mode

**Version:** Tuning + RC Noise Filter  
**Status:** Development / Tuning Phase

This project is a custom C++ Flight Controller firmware designed for the **Teensy 4.1** microcontroller. It implements a Self-Leveling (Stabilize) mode using an **MPU6050** IMU and the **Madgwick** sensor fusion algorithm.

> **⚠️ WARNING:** This firmware is currently configured for **PID Tuning (Roll Axis)**. Pitch and Yaw PIDs may be set to zero or conservative values. **Remove propellers** when testing on the bench.

## 🚀 Features
*   **High-Speed Loop:** Runs at ~400Hz (2500µs cycle time).
*   **Sensor Fusion:** Uses Madgwick filter for smooth attitude estimation (Roll/Pitch/Yaw).
*   **RC Noise Filtering:** Sanity checks on receiver inputs (900µs - 2100µs range) to prevent signal spikes.
*   **Safety Arming:** Simple throttle-threshold arming logic.
*   **Debug Ready:** Optimized for Arduino Serial Plotter (500,000 baud) to visualize PID performance.
*   **Custom Mixer:** Quad-X configuration.

## 🛠 Hardware Requirements
1.  **Microcontroller:** PJRC Teensy 4.1
2.  **IMU:** MPU6050 (I2C)
3.  **Receiver:** RC Receiver supporting PPM/PulsePosition (connected to one pin).
4.  **Frame:** Quadcopter X configuration.
5.  **ESCs:** Standard PWM ESCs (supports 250Hz update rate).

## 🔌 Pinout & Wiring

### Motor Layout (Quad X)
| Motor | Position | Direction | Teensy Pin |
| :--- | :--- | :--- | :--- |
| **M1** | Front Right | CCW (Counter-Clockwise) | **Pin 5** |
| **M2** | Back Right | CW (Clockwise) | **Pin 2** |
| **M3** | Back Left | CCW (Counter-Clockwise) | **Pin 3** |
| **M4** | Front Left | CW (Clockwise) | **Pin 4** |

### Peripherals
| Component | Connection | Teensy Pin |
| :--- | :--- | :--- |
| **RX Signal** | PulsePosition Input | **Pin 14** |
| **IMU SDA** | I2C Data | **Pin 18 (SDA0)** |
| **IMU SCL** | I2C Clock | **Pin 19 (SCL0)** |
| **Status LED** | Arming Status | **Pin 13 (Built-in)** |

## 📦 Libraries Required
Ensure you have the Teensyduino add-on installed in Arduino IDE.
*   **Wire** (Standard Arduino/Teensy library)
*   **PulsePosition** (Included with Teensyduino)

## ⚙️ Configuration & Tuning

### 1. PID Settings
Currently, the code is set up to tune the **ROLL** axis.
*   **Roll:** `Kp = 1.0`, `Ki = 0.5`, `Kd = 0.3`
*   **Pitch:** `Kp = 1.0`, `Ki = 0.5`, `Kd = 0.3` (Check code lines 22-25)
*   **Yaw:** Currently set to **0** in the provided snippet.

To change these, edit the `// ================= 2. PID TUNING (SETUP) =================` section in the source code.

### 2. RC Calibration
The code assumes standard PWM values:
*   **Min:** 1000us
*   **Mid:** 1500us
*   **Max:** 2000us
Ensure your transmitter endpoints are adjusted to match these values.

## 🎮 How to Use

1.  **Upload:** Flash the code to the Teensy 4.1 using Arduino IDE.
2.  **Calibration (Startup):**
    *   Place the drone on a **flat, level surface**.
    *   Plug in the battery/USB.
    *   **Do not move the drone** for the first 2-3 seconds. The code calculates the IMU Gyro/Accel offsets automatically during this time.
3.  **Arming:**
    *   Raise the Throttle stick above **1060µs** (approx 6-10% throttle).
    *   The Built-in LED (Pin 13) will turn **ON**.
    *   Motors will spin at idle speed.
4.  **Disarming:**
    *   Lower Throttle below 1060µs.
    *   Motors will stop and PIDs will reset.

## 📊 Debugging (Serial Plotter)
1.  Open **Arduino Serial Plotter**.
2.  Set Baud Rate to **500000**.
3.  The plotter will display:
    *   `Roll_Des` (Desired Angle) vs `Roll_IMU` (Actual Angle)
    *   `Pitch_Des` vs `Pitch_IMU`
    *   `Yaw_Des` vs `Yaw_IMU`

Use this visual data to tune your PIDs. If `IMU` oscillates around `Des`, lower P. If it's sluggish, increase P.

## 📝 Important Notes
*   **Safety First:** The code has a sanity check for RC inputs. If the receiver disconnects or sends values outside 900-2100us, the last valid value is held (or throttle acts safely based on logic). However, always set up a Failsafe on your RX/TX hardware.
*   **Loop Time:** The PID loop is hardcoded to `2500us` (400Hz).
*   **Voltage Levels:** The Teensy 4.1 is 3.3V logic. Ensure your receiver and ESC signal lines are compatible (most 5V ESCs work fine with 3.3V signals).

## ⚖️ Disclaimer
This firmware is for educational and hobbyist purposes. The author is not responsible for any damage to hardware, property, or injury caused by the use of this code. Fly responsibly.
