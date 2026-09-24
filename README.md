# Dual-Core ESP32 Self-Balancing Robot with Cascade PID & FreeRTOS

[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-Supported-green.svg)](https://www.freertos.org/)
[![Control Theory](https://img.shields.io/badge/Control_Theory-Cascade_PID-orange.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-ESP32-red.svg)]()

![ESP32 Self-Balancing Robot](images/Robot_Resmi.jpeg)

> **[▶️ Click Here to Watch the Demonstration Video on YouTube] https://youtu.be/P5MWFkgRvXQ**

## 📌 Project Overview
This repository contains the complete software and control architecture for a custom-built, two-wheeled self-balancing robot. The project demonstrates advanced embedded systems engineering, utilizing the **ESP32 dual-core processor** running **FreeRTOS** to isolate time-critical closed-loop control algorithms from asynchronous Wi-Fi telemetry tasks. 

The system relies on a custom **Cascade PID control loop**, sensor fusion via a **Complementary Filter**, and high-resolution **Hardware Timer Interrupts** to achieve seamless, vibration-free balancing and navigation.

## ⚙️ Hardware Architecture
The mechanical and electronic systems were custom-designed to ensure zero backlash and high torque delivery without unnecessary passive components.
*   **Microcontroller:** ESP32 Development Board
*   **IMU Sensor:** MPU6050 (6-DOF Accelerometer & Gyroscope)
*   **Actuators:** NEMA 17 Stepper Motors
*   **Motor Drivers:** DRV8825 (Configured for 1/8 microstepping, utilizing decoupling capacitors for stability without additional resistor networks)
*   **Communication:** Wi-Fi (UDP Protocol)

## 🧠 Software Architecture & RTOS
To guarantee absolute stability, the software architecture strictly separates deterministic control loops from non-deterministic network tasks using **FreeRTOS**.

*   **Core 1 (Real-Time Control):** Dedicated entirely to the 250 Hz Cascade PID loop and MPU6050 sensor fusion calculations.
*   **Core 0 (Telemetry & Comm):** Handles the Wi-Fi UDP server, processing incoming steering/throttle commands and streaming live state telemetry to a custom Python PC interface.
*   **Hardware Timer Interrupts (50 kHz):** The stepper motor pulses are generated in the background via a strict 50 kHz hardware timer ISR, completely offloading the step generation from the main RTOS tasks and eliminating jitter.

## 🧮 Advanced Control Algorithms
*   **Cascade PID Loop:** 
    *   *Inner Loop:* Fast-acting loop regulating the tilt angle based on raw gyroscope rates.
    *   *Outer Loop:* Slower loop regulating velocity and position, feeding setpoints to the inner loop.
*   **Sensor Fusion (Complementary Filter):** Fuses high-frequency gyroscope data (99%) with low-frequency accelerometer data (1%) to eliminate drift and mechanical noise.
*   **Dynamic Boot Calibration:** A custom initialization routine captures 300+ raw MPU6050 samples at startup to calculate and subtract static offsets and zero-rate drift, ensuring perfect vertical alignment regardless of temperature or manufacturing tolerances.

## 📂 Repository Structure
```text
├── src/
│   ├── main.cpp            # Main C++ source code with FreeRTOS tasks and ISR
│   └── robot_kontrol.py    # Python-based PC telemetry and control interface
├── 3D_Models/              # .stl files for the custom chassis and wheels
├── images/                 # Hardware photos and system diagrams
└── README.md
