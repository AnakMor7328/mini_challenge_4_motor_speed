# Mini Challenge 4 – Motor Speed Measurement using micro-ROS

## Overview

This project implements motor speed measurement for a DC motor using an incremental encoder connected to an ESP32. The encoder pulses are detected using hardware interrupts and used to estimate the motor speed in revolutions per minute (RPM). The measured speed is then published to ROS2 using micro-ROS.

The system also allows controlling the motor from ROS2 by sending PWM commands through a ROS topic. This enables real-time monitoring of the motor speed response using ROS visualization tools.

This challenge demonstrates the integration between:
- Embedded systems (ESP32)
- Hardware motor control
- Encoder feedback acquisition
- micro-ROS communication
- ROS2 visualization tools

---

## System Architecture

The system is composed of three main components:
1. **ROS2 Computer**
2. **micro-ROS Agent**
3. **ESP32 microcontroller** controlling the motor

Communication between ROS2 and the ESP32 is achieved through the micro-ROS agent over a serial interface.

