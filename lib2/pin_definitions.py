from pyb import Pin

# ENCODERS: Create timer objects for encoder counting
# Left encoder using Timer 1
encoder_left_timer = 1 
encoder_left_A = Pin.cpu.A8 
encoder_left_B = Pin.cpu.A9 

# Right encoder using Timer 2
encoder_right_timer = 2 
encoder_right_A = Pin.cpu.A0 
encoder_right_B = Pin.cpu.A1


# MOTORS: Create Pin objects for motor control
# Left motor
motor_left_enable = Pin.cpu.A7  # motor_left_enable on PA7
motor_left_dir = Pin.cpu.B6  # motor_left_dir on PB6
motor_left_timer = 3  # motor_left_timer is Timer 3
motor_left_channel = 2  # motor_left_channel is Timer Channel 3
motor_left_pwm = Pin.cpu.C7  # motor_left_pwm on PC7

# Right motor
motor_right_enable = Pin.cpu.A6  # motor_right_enable on PA6
motor_right_dir = Pin.cpu.B5  # motor_right_dir on PB5
motor_right_timer = 8  # motor_right_timer is Timer 8
motor_right_channel = 3  # motor_right_channel is Timer Channel 3
motor_right_pwm = pin=Pin.cpu.C8  # motor_right_pwm on PC8

# IR sensor
ir_ctrl = Pin.cpu.B2    # control pin for the IR sensor
ir_1 = Pin.cpu.A4       # IR Sensor #1
ir_3 = Pin.cpu.B0      # IR Sensor #2
ir_5 = Pin.cpu.C1      # IR Sensor #3
ir_7 = Pin.cpu.C0      # IR Sensor #4

# Blue Button 
blue_button = Pin.cpu.C13

# Bumper Switches
bump0 = Pin.cpu.B15
bump1 = Pin.cpu.B13
bump2 = Pin.cpu.B14
bump3 = Pin.cpu.B1
bump4 = Pin.cpu.B3
bump5 = Pin.cpu.B4
