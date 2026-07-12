# 🖱️ ESP32 BLE Smart Air Mouse (Single-Button Hybrid Control)

A high-performance, lag-free, and professional Air Mouse implementation using an **ESP32** and an **MPU6050 (Gyroscope/Accelerometer)** sensor. This project leverages the **BLE HID (Human Interface Device)** protocol to instantly emulate a real hardware mouse, offering seamless, all-time live pairing directly with mobile devices (Android/iOS) and PCs without needing any external companion applications.

Features an optimized **Kalman Filter** algorithm tailored for ultra-low latency, drift correction, and intelligent single-button gesture multiplexing (Click, Drag, Drop, and Scroll).

---

## 🚀 Key Features

* **Native BLE HID Stack:** Recognised by the host operating system instantly as a real Bluetooth Mouse. Stays persistently connected in the active input device layer (no ghosting or dropping to the saved device list).
* **Advanced Kalman Filtering:** Live sensor data processing to filter out ambient noise and human hand tremors, ensuring fluid and stable cursor navigation.
* **Auto-Calibration System:** Automatically samples and zeroes out hardware offsets (Sensor Drift) during the first 2 seconds of boot-up.
* **Single-Button Control Engine:** * **Hover/Move:** Tilt the controller without pressing the button to glide the cursor.
  * **Single Click:** A short press and release (<300ms) acts as a standard Left-Click to open apps/links.
  * **Drag & Scroll:** Holding the button down locks the left click, allowing you to drag items, swipe pages, or scroll documents naturally through hand gestures.

---

## 🗺️ System Architecture & Data Flow

Below is the technical schematic mapping how physical kinetic motion is transformed into standard Bluetooth HID packets:

[attachment_0](attachment)

```text
+------------------+       I2C (SDA/SCL)       +-----------------------+
|  MPU6050 Sensor  | ------------------------> |       ESP32 MCU       |
| (Accel + Gyro)   |                           |                       |
+------------------+                           |  1. Calibrate Offsets |
                                               |  2. Kalman Filter     |
+------------------+       GPIO 14 Interrupt   |  3. Resolution Mapper |
|   Smart Button   | ------------------------> |  4. Gesture Engine    |
+------------------+                           +-----------------------+
                                                           |
                                                           | Native BLE HID
                                                           v
                                               +-----------------------+
                                               |     Mobile/PC OS      |
                                               |   (Native Cursor)     |
                                               +-----------------------+
