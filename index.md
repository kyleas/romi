# Romi Time-Trials: ME405 Term Project

## Overview
This project is part of **ME 405** and focuses on **autonomous robot navigation** using a Romi chassis. The robot follows a predefined track while utilizing **PID control, IR sensors, and IMU-based navigation**. The goal is to complete time trials efficiently while demonstrating robustness in design and implementation.

## Features
- **Motor Control**: PID-based velocity control for left and right motors.
- **IR Sensor Navigation**: Line-following algorithm to track the predefined course.
- **IMU-Assisted Navigation**: Heading control for precise turns and orientation adjustments.
- **Finite-State Machine (FSM)**: Task-based execution for structured control flow.
- **Multitasking Framework**: Implemented using `cotask.py` and `task_share.py` for real-time control.

## System Architecture
The system is designed using **task-based multitasking**, where each component operates as a separate task:
1. **Task 1**: Handles user inputs and data logging.
2. **Task 2 & 3**: Left and right motor controllers using PID velocity control.
3. **Task 4**: Path-following logic using IR and IMU sensors.
4. **Task 5** (Optional): Emergency stop using bumper sensors.

## Code Structure
The project's Python code is organized as follows:

