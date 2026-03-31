# Self-Balancing-Robot
Self balancing robot using PID

## Project Overview
The core of this project is a discrete-time PID controller that processes data from an Inertial Measurement Unit (IMU) to calculate the necessary motor output. By balancing the "tilt" of the robot, the system compensates for gravity and external disturbances.

## Control Logic
The system utilizes three primary components to calculate the control signal u(t):
Proportional (K_p): Reacts to the current error (the difference between the desired setpoint and the actual tilt angle).
Integral (K_i): Accounts for past errors and eliminates steady-state offsets, ensuring the robot doesn't lean permanently in one direction.
Derivative (K_d): Predicts future error by analyzing the rate of change, providing a "damping" effect to prevent overshooting.

The ideal output is calculated as: u(t) = K_p*e(t) + K_i*e(t)dt + K_d* INT{de(t)/dt} 

## Features
Sensor Fusion: Implements a Complementary Filter (or Kalman Filter) to combine accelerometer and gyroscope data for an accurate, low-noise pitch angle.

PID Tuning: Modular constants (K_p, K_i, K_d) for easy calibration.

PWM Motor Control: Converts the PID output into Pulse Width Modulation signals for high-torque DC motors.

Sample Rate Optimization: Fixed-loop timing to ensure the dt component of the PID calculation remains constant for stability.

## Hardware Implementation
Microcontroller: (e.g., Arduino, ESP32, or STM32)
Sensors: MPU6050 (Accelerometer + Gyroscope)
Actuators: N20 or 550 geared motors with encoders

## Getting Started
Calibration: Place the robot in its perfectly upright "zero" position and run the calibration script to offset the IMU bias.
Tuning: Run Controls_Project_PID.ino. Start with K_i and K_d at zero. Increase K_p until the robot begins to oscillate, then introduce K_d to dampen the movement. Finally, use K_i to correct long-term drift.

