# Romi Time-Trials: ME405 Term Project

## Overview
Welcome to the **Romi Time-Trials Project**—a self-navigating robot designed for autonomous competition in **ME 405: Mechatronics**.  
The goal of this project is to build a **highly precise, efficient, and autonomous Romi robot** that can:
- **Follow a predefined track** using calibrated infrared (IR) sensors.
- **Adjust its heading** using an Inertial Measurement Unit (IMU).
- **Utilize PID control** for precise motor velocity regulation.
- **Respond to obstacles** with bumper sensors.

This project integrates real-time task scheduling, control theory, and embedded systems to create a functional and adaptable robotic platform.

---

## Features
- **Motor Control**: PID-based velocity control for both left and right motors.
- **IR Sensor Navigation**: Line-following algorithm to track the course.
- **IMU-Assisted Navigation**: Accurate heading control for smooth turns.
- **Finite-State Machine (FSM)**: Organized execution of robot behaviors.
- **Multitasking Framework**: Cooperative scheduling using `cotask.py` and shared data via `task_share.py`.

---

## System Architecture
The system is built on a task-based multitasking model, where each component operates as a separate task:

1. **Task 1**: Handles user inputs and data logging.
2. **Task 2 & Task 3**: Control left and right motors using PID velocity control.
3. **Task 4**: Implements path following using IR sensor and IMU data.
4. **Task 5** (Optional): Provides an emergency stop using bumper sensors.

*Diagram Placeholder*:  
_You can insert a system architecture diagram here to visually represent the tasks and their interactions._

---

## Code Structure
The project is organized into several modules:

**Main and Kinematics:**
- **`main.py`** – Main entry point; initializes tasks and the scheduler.
- **`romi.py`** – Handles robot kinematics, tracking position, heading, and distance.
- **`motor.py`** – Motor driver interface.
- **`pid.py`** – PID controller for motor speed regulation.
- **`imu.py`** – Interface to the IMU sensor.
- **`ir.py`** – IR sensor interface for line following.

**Task Scheduling and Data Sharing:**
- **`cotask.py`** – Cooperative task scheduler.
- **`task_share.py`** – Shared data and communication utilities.
- **`cqueue.py`** – Custom C-based queue implementations for efficiency.

**Sensors and Peripherals:**
- **`bump.py`** – Bumper switch interface.
- **`encoder.py`** – Quadrature encoder for velocity sensing.
- **`pin_definitions.py`** – Hardware pin mappings.

---