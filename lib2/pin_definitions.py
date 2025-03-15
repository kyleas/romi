"""!
@file pin_definitions.py
@brief Defines pin mappings and timer assignments for the ROMI hardware.
@details
  Centralizes all pin definitions for motors, encoders, IR sensors, bumpers,
  and the blue button. This makes it easier to maintain and modify hardware mappings.
"""

from pyb import Pin

# ------------------ ENCODER TIMERS & PINS ------------------
encoder_left_timer = 1
encoder_left_A = Pin.cpu.A8
encoder_left_B = Pin.cpu.A9

encoder_right_timer = 2
encoder_right_A = Pin.cpu.A0
encoder_right_B = Pin.cpu.A1

# ------------------ MOTOR PINS & TIMERS -------------------
# Left motor
motor_left_enable = Pin.cpu.A7     # PA7: motor enable for left motor
motor_left_dir = Pin.cpu.B6        # PB6: direction pin for left motor
motor_left_timer = 3               # Timer 3 used for left motor PWM
motor_left_channel = 2             # Timer channel
motor_left_pwm = Pin.cpu.C7        # PC7: PWM output pin for left motor

# Right motor
motor_right_enable = Pin.cpu.A6    # PA6: motor enable for right motor
motor_right_dir = Pin.cpu.B5       # PB5: direction pin for right motor
motor_right_timer = 8              # Timer 8 used for right motor PWM
motor_right_channel = 3            # Timer channel
motor_right_pwm = Pin.cpu.C8       # PC8: PWM output pin for right motor

# ------------------ IR SENSOR CONTROL & INPUTS ------------------
ir_ctrl = Pin.cpu.B2
ir_1 = Pin.cpu.A4
ir_3 = Pin.cpu.B0
ir_5 = Pin.cpu.C1
ir_7 = Pin.cpu.C0

# ------------------ BLUE BUTTON ------------------
blue_button = Pin.cpu.C13

# ------------------ BUMPER SWITCHES ------------------
bump0 = Pin.cpu.B15
bump1 = Pin.cpu.B13
bump2 = Pin.cpu.B14
bump3 = Pin.cpu.B1
bump4 = Pin.cpu.B3
bump5 = Pin.cpu.B4
