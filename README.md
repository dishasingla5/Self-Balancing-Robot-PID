# Self-Balancing Robot using PID Control

This project implements a real-time closed-loop control system for a two-wheeled self-balancing robot using MPU6050 IMU and PID control.

---

##  Features

- MPU6050 DMP-based orientation estimation
- PID control loop (~100 Hz)
- Stepper motor control using AccelStepper
- Deadband filtering for noise reduction
- Safety cutoff for fall detection
- Output clamping for stability

---

##  Working Principle

The robot behaves like an **inverted pendulum system**:

IMU → Roll Angle → PID Controller → Motor Speed → Balance Correction

---

##  Hardware

- Arduino (Uno/Nano)
- MPU6050 IMU
- Stepper Motors + Drivers
- Power Supply

---

##  Control Details

- Input: Roll angle (degrees)
- Setpoint: 0° (upright)
- Output: Motor speed
- Control frequency: ~100 Hz

---

##  Key Improvements

- Dead zone to reduce jitter
- Safety cutoff beyond ±30°
- Output saturation limits
- Startup delay for IMU stabilization

---

##  Future Work

- LQR control implementation
- Kalman filter-based estimation
- ROS 2 integration for autonomous navigation

---

##  Author

Disha Singla
