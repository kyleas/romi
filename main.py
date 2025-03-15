"""!
@file main.py
@brief Main execution file for the Romi autonomous navigation project.
@details 
   - Initializes tasks for motor control, path following, and bumper handling.  
   - Starts the scheduler to manage real-time control of the robot.  
   - Demonstrates line following, IMU-based heading control, and motor PID velocity control.

@author 
   - Toby Sagi
   - Kyle Schumacher

@date 2025
"""

import gc
import pyb
from array import array
from time import ticks_ms, ticks_diff, ticks_add

import lib2 as li
import lib2.cotask as cotask
import lib2.task_share as task_share

from lib2.pin_definitions import *
from lib2.encoder import Encoder
from lib2.motor import Motor
from lib2.pid import PID
from lib2.ir import IR
from lib2.imu import IMU
from lib2.romi import Romi
from lib2.bump import Bump
from lib2.line_follow import LineFollow


def task1_main(shares):
    """!
    @brief Task which controls user inputs and data logging.
    @details 
      - Initializes motor control parameters.
      - Collects positional/velocity data from motors.
      - Logs data at each iteration until the run finishes.

    @param shares 
      A tuple of references to shares and queues used by this task:
      - share_motor_controller_period : The sampling period for motor control (ms).
      - share_left_effort            : Reference to the left motor effort share.
      - share_right_effort           : Reference to the right motor effort share.
      - share_left_run_dur           : Left motor run duration (ms).
      - share_left_run_dur           : (Repeated) Right motor run duration (ms).
      - queue_left_position          : Queue storing left motor positions.
      - queue_right_position         : Queue storing right motor positions.
      - queue_left_velocity          : Queue storing left motor velocities.
      - queue_right_velocity         : Queue storing right motor velocities.
      - share_cur_left_vel           : Current left motor velocity share.
      - share_cur_right_vel          : Current right motor velocity share.
    """
    # Unpack shares and queues
    share_motor_controller_period, \
    share_left_effort, \
    share_right_effort, \
    share_left_run_dur, \
    share_left_run_dur, \
    queue_left_position, \
    queue_right_position, \
    queue_left_velocity, \
    queue_right_velocity, \
    share_cur_left_vel, \
    share_cur_right_vel = shares

    # State machine definitions
    state = 0
    S0_INIT_PARAMS = 0
    S1_INIT_DATA_STORAGE = 1
    S2_RUN_AND_COLLECT_DATA = 2
    S3_PRINT_RESULTS = 3
    S4_HOLD = 4

    effort = 0
    run_dur = 0

    while True:
        if state == S0_INIT_PARAMS:
            print("T1 S0 - Initialize parameters.")
            # Example motor effort and duration
            effort = 25
            run_dur = 2000  # ms
            state = S1_INIT_DATA_STORAGE
            yield state

        elif state == S1_INIT_DATA_STORAGE:
            print("T1 S1 - Preparing data storage.")
            # Calculate how many data points to store
            points = int(run_dur / share_motor_controller_period.get()) + 1

            # Pre-allocate arrays (just a demonstration; commented out usage below)
            data_L_pos = array('H', points * [0])
            data_R_pos = array('H', points * [0])
            data_L_vel = array('f', points * [0])
            data_R_vel = array('f', points * [0])

            idx = 0

            # Set shared variables for motor efforts and durations
            share_left_effort.put(effort)
            share_right_effort.put(effort)
            share_left_run_dur.put(run_dur)
            share_right_run_dur.put(run_dur)

            state = S2_RUN_AND_COLLECT_DATA
            yield state

        elif state == S2_RUN_AND_COLLECT_DATA:
            # Continually check if motor run is finished & queue is empty
            if (share_left_run_dur.get() == 0 and share_right_run_dur.get() == 0
                and share_left_effort.get() == 0 and share_right_effort.get() == 0
                and queue_left_position.empty() and queue_right_position.empty()
                and queue_left_velocity.empty() and queue_right_velocity.empty()):

                state = S3_PRINT_RESULTS
                yield state
            else:
                # Example data retrieval (commented out in user code)
                # data_L_pos[idx] = queue_left_position.get()
                # data_R_pos[idx] = queue_right_position.get()
                # data_L_vel[idx] = queue_left_velocity.get()
                # data_R_vel[idx] = queue_right_velocity.get()
                idx += 1
                yield state

        elif state == S3_PRINT_RESULTS:
            print("T1 S3 - Printing results (example).")
            # Example printing of stored data (commented out in user code)
            # for i in range(len(data_L_vel)):
            #     print(f"{i / points * (run_dur / 1000)}, "
            #           f"{data_L_pos[i]}, {data_R_pos[i]}, "
            #           f"{data_L_vel[i]}, {data_R_vel[i]}")
            state = S4_HOLD
            yield state

        elif state == S4_HOLD:
            # Idle / hold state
            yield state

        else:
            state = S0_INIT_PARAMS
            raise RuntimeError("State machine in task1_main is in an illegal state. Resetting to S0.")


def task2_left_motor_controller(shares):
    """!
    @brief Task which controls the left motor using PID velocity control.
    @details 
      - Creates a Motor and Encoder object for the left motor.  
      - Waits for a start signal before enabling the motor.  
      - Once running, updates encoder feedback and computes PID effort to maintain set velocity.

    @param shares 
      A tuple of references to the following:
      - share_run_dur     : Duration to run the motor (ms).
      - share_left_vel    : Desired left motor velocity (counts/second or similar).
      - share_max_eff     : Maximum allowed effort for the motor (unused in original code).
      - share_cur_left_vel: Current left motor velocity measured by the encoder.
      - share_wait        : A flag (0 or 1) that indicates when to start the motor.
      - share_stop        : A flag (0 or 1) that indicates when to stop the motor.
    """
    (share_run_dur, 
     share_left_vel, 
     share_max_eff, 
     share_cur_left_vel, 
     share_wait, 
     share_stop) = shares

    # State machine definitions
    state = 0
    S0_CREATE_OBJECTS = 0
    S1_STANDBY = 1
    S2_RUN = 2

    while True:
        if state == S0_CREATE_OBJECTS:
            print("T2 Left Motor - Creating objects.")
            Motor_Left = Motor(motor_left_enable, motor_left_dir, 
                               motor_left_timer, motor_left_channel, 
                               motor_left_pwm)
            Encoder_Left = Encoder(encoder_left_timer, encoder_left_A, encoder_left_B)
            # Tuned PID gains for left motor
            PID_Motor_Left = PID(0.3, 0.009, 0, 
                                 100 / 34.8 / 7.2, 
                                 0.44 / 7.2 * 100, 
                                 integral_error_max=500)

            state = S1_STANDBY
            yield state

        elif state == S1_STANDBY:
            # Wait for the share_wait flag to go low (0) to start
            if share_wait.get() == 0:
                print("T2 Left Motor - Wait is over, enabling motor.")
                Motor_Left.enable()
                state = S2_RUN
            yield state

        elif state == S2_RUN:
            # Update encoder reading
            Encoder_Left.update()
            current_velocity = Encoder_Left.get_velocity()
            share_cur_left_vel.put(current_velocity)

            # Compute new motor effort via PID
            effort = PID_Motor_Left.update(share_left_vel.get(), current_velocity)
            # Saturate effort between -100 and 100
            effort = max(min(effort, 100), -100)
            Motor_Left.set_effort(effort)

            # Check if we should stop
            if share_stop.get() == 1:
                print("T2 Left Motor - Stop signal received, disabling motor.")
                Motor_Left.disable()
                Motor_Left.set_effort(0)
                share_run_dur.put(0)
                share_left_vel.put(0)
            yield state

        else:
            # Error handling
            state = S1_STANDBY
            print("T2 Error - Left Motor Controller")
            raise RuntimeError("Task2_left_motor_controller in illegal state. Reset to S1.")


def task3_right_motor_controller(shares):
    """!
    @brief Task which controls the right motor using PID velocity control.
    @details 
      - Similar to the left motor controller, but for the right motor.  
      - Creates Motor and Encoder objects and uses PID to maintain the desired velocity.

    @param shares 
      A tuple of references to:
      - share_run_dur      : Duration to run the motor (ms).
      - share_right_vel    : Desired right motor velocity.
      - share_cur_right_vel: Current right motor velocity measured by the encoder.
      - share_wait         : A flag indicating when to start the motor.
      - share_stop         : A flag indicating when to stop the motor.
    """
    (share_run_dur,
     share_right_vel,
     share_cur_right_vel,
     share_wait,
     share_stop) = shares
    
    # State machine definitions
    state = 0
    S0_CREATE_OBJECTS = 0
    S1_STANDBY = 1
    S2_RUN = 2

    while True:
        if state == S0_CREATE_OBJECTS:
            print("T3 Right Motor - Creating objects.")
            Motor_Right = Motor(motor_right_enable, motor_right_dir, 
                                motor_right_timer, motor_right_channel, 
                                motor_right_pwm)
            Encoder_Right = Encoder(encoder_right_timer, encoder_right_A, encoder_right_B)
            # Tuned PID gains for right motor
            PID_Motor_Right = PID(0.3, 0.009, 0, 
                                  100 / 33.7 / 7.2, 
                                  0.49 / 7.2 * 100, 
                                  integral_error_max=500)
            state = S1_STANDBY
            yield state

        elif state == S1_STANDBY:
            if share_wait.get() == 0:
                print("T3 Right Motor - Wait is over, enabling motor.")
                Motor_Right.enable()
                state = S2_RUN
            yield state

        elif state == S2_RUN:
            # Update encoder reading
            Encoder_Right.update()
            current_velocity = Encoder_Right.get_velocity()
            share_cur_right_vel.put(current_velocity)

            # Compute PID effort
            effort = PID_Motor_Right.update(share_right_vel.get(), current_velocity)
            effort = max(min(effort, 100), -100)
            Motor_Right.set_effort(effort)

            # Check for stop
            if share_stop.get() == 1:
                print("T3 Right Motor - Stop signal received, disabling motor.")
                Motor_Right.disable()
                Motor_Right.set_effort(0)
                share_run_dur.put(0)
                share_right_vel.put(0)
            yield state

        else:
            state = S1_STANDBY
            raise RuntimeError("Task3_right_motor_controller in illegal state. Reset to S1.")


def task4_path_follow(shares):
    """!
    @brief Task which controls path following for the Romi.
    @details 
      - Creates IMU and IR objects, calibrates sensors, and then follows a preset path.  
      - Uses scheduled mode changes (IR line following, heading hold, turn, etc.) based on distance traveled.  
      - Utilizes a finite state machine for initialization, calibration, and run modes.

    @param shares 
      A tuple of references to:
      - share_chasssis_vel : Desired chassis velocity (m/s or other unit).
      - share_left_vel     : Desired left motor velocity share (output of path logic).
      - share_right_vel    : Desired right motor velocity share (output of path logic).
      - share_cur_left_vel : Current left motor velocity.
      - share_cur_right_vel: Current right motor velocity.
      - share_wait         : A flag for waiting before starting.
      - share_mode         : Holds the current mode (IR, HEADING, TURN, etc.).
      - share_heading      : Desired heading for heading or turning modes.
      - share_continue     : A flag to let path follow logic continue after certain maneuvers.
    """
    (share_chasssis_vel,
     share_left_vel,
     share_right_vel,
     share_cur_left_vel,
     share_cur_right_vel,
     share_wait,
     share_mode,
     share_heading,
     share_continue) = shares
    
    # State machine definitions
    state = 0
    S0_INIT = 0
    S1_CALIBRATE = 1
    S2_RUN = 2

    # Mode definitions
    MODE_HEADING = 0
    MODE_IR = 1
    MODE_TURN = 2
    MODE_BACKWARD = 3
    MODE_STOP = 4

    # Example scheduled modes based on distance
    mode_schedule = [
        (0.00, MODE_IR),
        (0.9, MODE_HEADING, 90),
        (1.0, MODE_IR),
        (3.70, MODE_HEADING, 179),
        (4.42, MODE_TURN, 270),
        (4.30, MODE_HEADING, 270),
        (4.5, MODE_IR),
    ]

    mode_schedule2 = [
        (0.00, MODE_BACKWARD, 0.02),
        (-0.10, MODE_TURN, 0),
        (-0.10, MODE_HEADING, 0),
        (0.3, MODE_TURN, 270),
        (0.3, MODE_HEADING, 270),
        (0.53, MODE_TURN, 180),
        (0.53, MODE_IR),
        (1, MODE_STOP),
    ]

    button = pyb.Pin(blue_button, pyb.Pin.IN, pyb.Pin.PULL_UP)
    bumper_status = 0
    share_continue.put(1)
    turn_gen = None

    mode_index = 0
    test_mode = 0

    while True:
        if state == S0_INIT:
            print("T4 Path Follow - Initializing objects.")
            imu_obj = IMU()                   # IMU for heading
            PID_Omega = PID(0.08, 0.00, 0.0)  # PID for angular control
            chassis = Romi()                  # Romi chassis for distance/heading
            ir_sensor = IR(ir_ctrl, [ir_1, ir_3, ir_5, ir_7])
            bumper = Bump(bump0, bump1, bump2, bump3, bump4, bump5)
            line_follower = LineFollow(ir_sensor)

            share_wait.put(1)
            print("IMU object created.")
            state = S1_CALIBRATE
            yield state

        elif state == S1_CALIBRATE:
            # IMU calibration
            while imu_obj.startup() == False:
                for i in range(1000):
                    yield state

            # Default IR sensor calibration
            ir_sensor.default_calibration()

            print("Cal done. Press the blue button to start")
            while button.value() == 1:
                yield state

            imu_obj.set_mode(IMU.NDOF)
            chassis.zero_heading(imu_obj.read_euler_heading())
            if test_mode == 1:
                chassis.zero_heading(imu_obj.read_euler_heading() + 90)

            state = S2_RUN
            share_wait.put(0)
            share_chasssis_vel.put(5 * 0.036)  # Example conversion factor
            for i in range(5):
                yield state
            chassis.update_time()
            yield state

        elif state == S2_RUN:
            # Check bumper
            bumper.update()
            if bumper.get() and bumper_status == 0:
                bumper_status = 1
                chassis.zero_distance()
                mode_index = 0
                share_continue.put(1)
                turn_gen = None
                print("Bumper pressed, switching mode schedules.")

            dist = chassis.get_distance()
            print(f"Distance: {dist}")

            # Switch modes based on distance traveled
            if bumper_status == 0:
                if mode_index < len(mode_schedule) and dist >= mode_schedule[mode_index][0]:
                    entry = mode_schedule[mode_index]
                    new_mode = entry[1]
                    share_mode.put(new_mode)
                    print(f"Switching to mode: {new_mode}")
                    if len(entry) > 2:
                        share_heading.put(entry[2])
                        print(f"Setting heading: {entry[2]}")
                    mode_index += 1
            else:
                if (mode_index < len(mode_schedule2) 
                    and dist >= mode_schedule2[mode_index][0] 
                    and share_continue.get() == 1):
                    entry = mode_schedule2[mode_index]
                    new_mode = entry[1]
                    share_mode.put(new_mode)
                    print(f"Switching to mode: {new_mode}")
                    if len(entry) > 2:
                        share_heading.put(entry[2])
                        print(f"Setting heading: {entry[2]}")
                    mode_index += 1

            current_mode = share_mode.get()

            # Respond to current mode
            if current_mode == MODE_IR:
                follow_line(chassis, line_follower,
                            share_cur_left_vel, share_cur_right_vel,
                            share_left_vel, share_right_vel,
                            share_max_vel)

            elif current_mode == MODE_HEADING:
                heading = share_heading.get()
                follow_heading(chassis, imu_obj, PID_Omega,
                               share_chasssis_vel,
                               share_cur_left_vel, share_cur_right_vel,
                               share_left_vel, share_right_vel,
                               share_max_vel, heading)

            elif current_mode == MODE_TURN:
                turn_angle = share_heading.get()
                if turn_gen is None:
                    turn_gen = follow_turn(chassis, imu_obj, PID_Omega,
                                           share_left_vel, share_right_vel,
                                           share_max_vel, turn_angle, share_continue)
                try:
                    next(turn_gen)
                except StopIteration:
                    print("Turn completed.")
                    turn_gen = None

            elif current_mode == MODE_BACKWARD:
                if turn_gen is None:
                    turn_gen = go_backward(chassis, share_left_vel, share_right_vel,
                                           share_cur_left_vel.get(), share_cur_right_vel.get(),
                                           chassis.get_angle(), share_heading.get(),
                                           share_continue)
                try:
                    next(turn_gen)
                except StopIteration:
                    print("Backward completed.")
                    turn_gen = None

            elif current_mode == MODE_STOP:
                raise KeyboardInterrupt

            yield state

        else:
            state = S0_INIT
            raise RuntimeError("Task4_path_follow in an invalid state. Resetting to S0.")


def follow_line(chassis, line_follower, cur_left_vel_share, cur_right_vel_share,
                left_vel_share, right_vel_share, max_vel_share):
    """!
    @brief IR-based line following helper function.
    @details 
      - Uses IR sensor readings to adjust left/right wheel velocities.  
      - Updates the chassis object with current velocities for tracking.

    @param chassis        : Romi chassis object to track distance, heading, etc.
    @param line_follower  : LineFollow object handling IR sensor feedback.
    @param cur_left_vel_share  : Shared current left motor velocity.
    @param cur_right_vel_share : Shared current right motor velocity.
    @param left_vel_share : Shared desired left motor velocity (to be updated by this function).
    @param right_vel_share: Shared desired right motor velocity (to be updated by this function).
    @param max_vel_share  : Shared maximum velocity for line following.
    """
    cur_left_vel = cur_left_vel_share.get()
    cur_right_vel = cur_right_vel_share.get()
    chassis.update(cur_left_vel, cur_right_vel, chassis.get_angle())

    left_vel, right_vel = line_follower.update(max_vel_share)
    left_vel_share.put(left_vel)
    right_vel_share.put(right_vel)


def go_backward(chassis, left_vel_share, right_vel_share,
                cur_left_vel, cur_right_vel, cur_heading,
                distance, continue_val):
    """!
    @brief Moves the robot backward by a certain distance.
    @details 
      - Zeroes distance, then drives in reverse until distance traveled is met.  
      - Sets wheel velocities to negative values, and stops when the condition is reached.

    @param chassis        : Romi chassis object.
    @param left_vel_share : Shared left motor velocity control.
    @param right_vel_share: Shared right motor velocity control.
    @param cur_left_vel   : Current left motor velocity (unused).
    @param cur_right_vel  : Current right motor velocity (unused).
    @param cur_heading    : Current heading of the chassis.
    @param distance       : Target distance to move backward (some distance metric).
    @param continue_val   : Shared flag to resume path follow after backward motion.
    """
    chassis.zero_distance()
    continue_val.put(0)
    print("Going backward.")
    while True:
        left_vel_share.put(-20)
        right_vel_share.put(-20)
        chassis.update(left_vel_share.get(), right_vel_share.get(), cur_heading)

        error = distance + chassis.get_distance()
        if error < 0.001:
            print("Backward movement complete.")
            left_vel_share.put(0)
            right_vel_share.put(0)
            chassis.zero_distance()
            chassis.update_time()
            continue_val.put(1)
            break

        yield


def follow_heading(chassis, imu_obj, PID_Omega, chassis_vel_share,
                   cur_left_vel_share, cur_right_vel_share,
                   left_vel_share, right_vel_share,
                   max_vel_share, target_heading):
    """!
    @brief Maintains a specific heading using IMU feedback and PID control.
    @details 
      - Reads the current heading from the IMU.  
      - Computes heading error and uses PID to calculate angular velocity (omega).  
      - Converts chassis velocity and chassis_omega into left/right velocities.

    @param chassis           : Romi chassis object.
    @param imu_obj           : IMU object for heading data.
    @param PID_Omega         : PID controller for angular velocity.
    @param chassis_vel_share : Desired forward velocity of the chassis.
    @param cur_left_vel_share: Current left motor velocity.
    @param cur_right_vel_share: Current right motor velocity.
    @param left_vel_share    : Shared left motor velocity to be updated.
    @param right_vel_share   : Shared right motor velocity to be updated.
    @param max_vel_share     : Shared maximum velocity possible.
    @param target_heading    : Target heading in degrees.
    """
    chassis_vel = chassis_vel_share.get()
    current_heading = imu_obj.read_euler_heading()

    cur_left_vel = cur_left_vel_share.get()
    cur_right_vel = cur_right_vel_share.get()
    chassis.update(cur_left_vel, cur_right_vel, current_heading)

    # Calculate heading error (normalize to [-180, 180])
    error = target_heading - chassis.get_angle()
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    chassis_omega = -1 * PID_Omega.update_error(error)
    left_vel, right_vel = chassis.calc_des_wheel_vel(chassis_vel, chassis_omega, max_vel_share)
    left_vel_share.put(left_vel)
    right_vel_share.put(right_vel)


def follow_turn(chassis, imu_obj, PID_Omega, left_vel_share, right_vel_share,
                max_vel_share, turn_angle, continue_val):
    """!
    @brief Implements an in-place turn to a specific heading.
    @details 
      - Uses a generator approach to step through the turn each iteration.  
      - Zero linear velocity and apply an angular velocity until the desired heading is reached.

    @param chassis       : Romi chassis object.
    @param imu_obj       : IMU object to measure heading.
    @param PID_Omega     : PID controller for angular velocity.
    @param left_vel_share: Shared left motor velocity to be updated.
    @param right_vel_share: Shared right motor velocity to be updated.
    @param max_vel_share : Shared max velocity for limiting turning speed.
    @param turn_angle    : Relative target heading to turn to (e.g., +90 deg).
    @param continue_val  : Shared flag set to 1 when turn is complete.
    """
    print("Initiating turn to angle:", turn_angle)
    target_heading = turn_angle
    initial_distance = chassis.get_distance()
    continue_val.put(0)

    while True:
        current_heading = imu_obj.read_euler_heading()
        chassis.update_angle(current_heading)

        error = target_heading - chassis.get_angle()
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < 3:
            print("Turn complete at heading:", chassis.get_angle())
            left_vel_share.put(0)
            right_vel_share.put(0)
            chassis.set_distance(initial_distance)
            chassis.update_time()
            continue_val.put(1)
            break

        chassis_omega = -1 * PID_Omega.update_error(error)
        left_vel, right_vel = chassis.calc_des_wheel_vel(0, chassis_omega, max_vel_share)
        left_vel_share.put(left_vel)
        right_vel_share.put(right_vel)

        yield


def task5_bumper():
    """!
    @brief Simple bumper task to demonstrate emergency stop or exit.
    @details 
      - Monitors the bump sensors and raises KeyboardInterrupt when pressed.

    @exception KeyboardInterrupt 
      Raises KeyboardInterrupt when a bumper is pressed, causing the main scheduler to stop.
    """
    state = 0
    S0_INIT = 0
    S1_WAIT_FOR_PRESS = 1
    S2_CRASH_OUT = 2

    while True:
        if state == S0_INIT:
            bumper = Bump(bump0, bump1, bump2, bump3, bump4, bump5)
            state = S1_WAIT_FOR_PRESS
            yield state

        elif state == S1_WAIT_FOR_PRESS:
            bumper.update()
            if bumper.get():
                print("Bumper pressed")
                state = S2_CRASH_OUT
            yield state

        elif state == S2_CRASH_OUT:
            raise KeyboardInterrupt

        else:
            state = S0_INIT
            raise RuntimeError("task5_bumper in invalid state. Reset to S0.")


if __name__ == "__main__":
    """!
    @brief Entry point for running the Romi robot control system.
    @details 
      - Sets up shared variables, queues, and tasks.  
      - Adds tasks to a cooperative scheduler (`cotask`) and runs them until interrupted.  
      - On KeyboardInterrupt, stops the motors and prints diagnostic data.
    """
    print("Testing Lab_0x03 Motor Controllers with Multitasking using cotask.py and task_share.py\n"
          "Press Ctrl-C to stop and show diagnostics.")

    # Create shares and queues for demonstration
    share_motor_controller_period = task_share.Share('i', thread_protect=False, 
                                                     name="Share_Left_Motor_Controller_Period")
    share_motor_controller_period.put(20)

    share_left_effort = task_share.Share('i', thread_protect=False, name="Share_Left_Effort")
    share_left_effort.put(0)

    share_left_run_dur = task_share.Share('i', thread_protect=False, name="Share_Left_Run_Dur")
    share_left_run_dur.put(5000000)

    share_right_effort = task_share.Share('i', thread_protect=False, name="Share_Right_Effort")
    share_right_effort.put(0)

    share_right_run_dur = task_share.Share('i', thread_protect=False, name="Share_Right_Run_Dur")
    share_right_run_dur.put(5000000)

    q_len = 500
    queue_left_position = task_share.Queue('l', q_len, thread_protect=False, 
                                           overwrite=False, name="Queue_Left_Position")
    queue_left_velocity = task_share.Queue('f', q_len, thread_protect=False, 
                                           overwrite=False, name="Queue_Left_Velocity")
    queue_right_position = task_share.Queue('l', q_len, thread_protect=False, 
                                            overwrite=False, name="Queue_Right_Position")
    queue_right_velocity = task_share.Queue('f', q_len, thread_protect=False, 
                                            overwrite=False, name="Queue_Right_Velocity")

    share_max_vel = task_share.Share('f', thread_protect=False, name="Share_Max_Vel")
    share_max_vel.put(40)  # Example maximum velocity

    share_max_eff = task_share.Share('f', thread_protect=False, name="Share_Max_Eff")
    share_max_eff.put(70)

    share_right_vel = task_share.Share('f', thread_protect=False, name="Share_Right_Vel")
    share_right_vel.put(0)

    share_left_vel = task_share.Share('f', thread_protect=False, name="Share_Left_Vel")
    share_left_vel.put(0)

    share_dir = task_share.Share('i', thread_protect=False, name="Share_Dir")
    share_dir.put(0)

    share_chassis_vel = task_share.Share('f', thread_protect=False, name="Share_Chassis_Vel")
    share_chassis_vel.put(0)

    share_cur_left_vel = task_share.Share('f', thread_protect=False, name="Share_Cur_Left_Vel")
    share_cur_left_vel.put(0)

    share_cur_right_vel = task_share.Share('f', thread_protect=False, name="Share_Cur_Right_Vel")
    share_cur_right_vel.put(0)

    share_wait = task_share.Share('i', thread_protect=False, name="Share_Wait")
    share_wait.put(1)

    share_stop = task_share.Share('i', thread_protect=False, name="Share_Stop")
    share_stop.put(0)

    share_mode = task_share.Share('i', thread_protect=False, name="Share_Mode")
    share_mode.put(0)

    share_heading = task_share.Share('f', thread_protect=False, name="Share_Heading")
    share_heading.put(0)

    share_continue = task_share.Share('i', thread_protect=False, name="Share_Continue")
    share_continue.put(1)

    # Create tasks for each operational function
    task1 = cotask.Task(
        task1_main, name="Task_1", priority=1, period=30,
        profile=True, trace=False,
        shares=(
            share_motor_controller_period, share_left_effort, share_right_effort,
            share_left_run_dur, share_left_run_dur, queue_left_position,
            queue_right_position, queue_left_velocity, queue_right_velocity,
            share_cur_left_vel, share_cur_right_vel
        )
    )

    task2 = cotask.Task(
        task2_left_motor_controller, name="Task_2", priority=2,
        period=share_motor_controller_period.get(), profile=True, trace=False,
        shares=(
            share_left_run_dur, share_left_vel, share_max_eff,
            share_cur_left_vel, share_wait, share_stop
        )
    )

    task3 = cotask.Task(
        task3_right_motor_controller, name="Task_3", priority=2,
        period=share_motor_controller_period.get(), profile=True, trace=False,
        shares=(
            share_right_run_dur, share_right_vel, share_cur_right_vel,
            share_wait, share_stop
        )
    )

    task4 = cotask.Task(
        task4_path_follow, name="Task_4", priority=3, period=200,
        profile=True, trace=False,
        shares=(
            share_chassis_vel, share_left_vel, share_right_vel,
            share_cur_left_vel, share_cur_right_vel, share_wait,
            share_mode, share_heading, share_continue
        )
    )

    task5 = cotask.Task(
        task5_bumper, name="Task_5", priority=4, period=1000
    )

    # Add chosen tasks to the task list
    # cotask.task_list.append(task1)  # Uncomment if you wish to run Task 1
    cotask.task_list.append(task2)
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
    # cotask.task_list.append(task5)  # Uncomment if you wish to run Task 5

    # Run garbage collector before starting real-time scheduler
    gc.collect()

    # Start the cooperative multitasking scheduler
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            # On interrupt, disable motors and exit
            Motor_Right = Motor(motor_right_enable, motor_right_dir,
                                motor_right_timer, motor_right_channel,
                                motor_right_pwm)
            Motor_Left = Motor(motor_left_enable, motor_left_dir,
                               motor_left_timer, motor_left_channel,
                               motor_left_pwm)
            Motor_Left.disable()
            Motor_Left.set_effort(0)
            Motor_Right.disable()
            Motor_Right.set_effort(0)

            print("Please I'm stopping please")
            break

    # Print diagnostics for tasks and shared variables
    print('\n' + str(cotask.task_list))
    print(task_share.show_all())
    print('')
