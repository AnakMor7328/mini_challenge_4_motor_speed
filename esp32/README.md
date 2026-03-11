# Mini Challenge 4 – Motor Speed Measurement using micro-ROS

## Overview
This project implements motor speed measurement for a DC motor using an encoder connected to an ESP32. Encoder pulses are detected with interrupts and used to estimate RPM, which is published to ROS2 via micro-ROS. The motor can also be controlled from ROS2 by sending PWM commands.

## Hardware Connections
- **L298N**: PWM→GPIO14, IN1→GPIO26, IN2→GPIO15
- **Encoder**: Phase A→GPIO19, Phase B→GPIO5

## How It Works
- Subscribes to `/cmd_pwm` (Float32, range -1 to 1) to control motor direction and speed
- Encoder interrupt on Phase A counts pulses
- Every 100ms, calculates RPM = (delta_counts/500) / 0.1 * 60
- Publishes RPM to `/motor_speed_y` (Float32)

## Running the System
```bash
# Terminal 1 - Start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Terminal 2 - Send motor command
ros2 topic pub /cmd_pwm std_msgs/msg/Float32 "{data: 0.3}" --once

# Terminal 3 - Monitor speed
ros2 topic echo /motor_speed_y

# Terminal 4 - Visualize
ros2 run rqt_plot rqt_plot  # Add /motor_speed_y/data
