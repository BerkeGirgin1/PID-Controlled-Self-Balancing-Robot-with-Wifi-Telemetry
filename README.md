# Dual-Core ESP32 Self-Balancing Robot with Cascade PID & FreeRTOS

[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-Supported-green.svg)](https://www.freertos.org/)
[![Control Theory](https://img.shields.io/badge/Control_Theory-Cascade_PID-orange.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-ESP32-red.svg)]()

<img width="1878" height="2272" alt="Robot_Resmi" src="https://github.com/user-attachments/assets/8311c2ab-420d-4730-96bb-0f4ea3efccbd" />

> ▶️ [Demonstration video on YouTube](https://youtu.be/P5MWFkgRvXQ)

## 📌 Project Overview
This repository contains the software and control architecture for a custom-built, two-wheeled self-balancing robot (graduation project). The ESP32's two cores run **FreeRTOS** tasks that keep the time-critical balance loop separate from the asynchronous Wi-Fi telemetry.

The system uses a **cascade PID control loop**, sensor fusion with a **complementary filter**, and a **hardware timer interrupt** for step generation, so the robot can balance and drive under remote control.

## ⚙️ Hardware Architecture
The chassis and wheels were custom-designed (STL files included).
*   **Microcontroller:** ESP32 Development Board
*   **IMU Sensor:** MPU6050 (6-DOF Accelerometer & Gyroscope)
*   **Actuators:** NEMA 17 Stepper Motors
*   **Motor Drivers:** DRV8825 (configured for 1/8 microstepping, with decoupling capacitors)
*   **Communication:** Wi-Fi (UDP)

## 🧠 Software Architecture & RTOS
To keep control-loop timing deterministic, the software separates the control loop from network tasks using **FreeRTOS**.

*   **Core 1 (Real-Time Control):** Runs the 250 Hz (4 ms, `vTaskDelayUntil`) balance task: IMU read, filtering, sensor fusion and both PID loops.
*   **Core 0 (Telemetry & Comm):** Runs the Wi-Fi UDP server, processes incoming steering/throttle commands and streams live state telemetry to a Python PC interface.
*   **Hardware Timer Interrupt (50 kHz):** Stepper pulses are generated in a 50 kHz timer ISR, which keeps step timing independent of the RTOS tasks.

## 🧮 Control Algorithms
*   **Cascade PID Loop (both loops at 250 Hz):**
    *   *Inner loop:* PD controller on tilt angle, using the gyro rate as the derivative term.
    *   *Outer loop:* PI controller on estimated wheel speed; its output is the tilt setpoint for the inner loop.
*   **Sensor Fusion (Complementary Filter):** Fuses gyroscope (99%) and accelerometer (1%) angle estimates after a 5-sample moving average, to limit drift and mechanical noise.
*   **Boot Calibration:** At startup, 300 raw MPU6050 samples are averaged to remove static sensor offsets. While the robot is still, a runtime gyro-bias tracker keeps compensating temperature drift.
*   **Fall Detection:** If the tilt stays above 40° for 25 control cycles, the motors stop and the robot enters a fault state; it resumes balancing once it is brought back upright.
*   **Live Tuning:** PID gains can be changed at runtime over UDP (`P`, `D`, `T`, `I` commands) without reflashing.

## 📂 Repository Structure
```text
├── src/
│   ├── main.cpp                         # FreeRTOS tasks, control loops and step ISR
│   └── robot_kontrol.py                 # Python PC telemetry and control interface
├── mechanical_design.zip                # STL files: chassis (berke_case) and wheel (Wheel_v4)
├── Final_Project_Poster_BerkeGirgin.pdf # Graduation project poster
└── README.md
```