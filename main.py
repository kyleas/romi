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
  Task which controls user inputs and date logging
  @param shares A list holding the share(s) and queue(s) used by this task
  """
  # Get references to the share(s) and queue(s) which have been passed to this task
  share_motor_controller_period, share_left_effort, share_right_effort, share_left_run_dur, share_left_run_dur, queue_left_position, queue_right_position, queue_left_velocity, queue_right_velocity, share_cur_left_vel, share_cur_right_vel = shares

  # State machine variables
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
      print("T1 S0")
      effort = 25
      run_dur = 2_000
      state = 1
      yield (state)

    elif state == S1_INIT_DATA_STORAGE:
      print("T1 S1")
      points = int(run_dur / share_motor_controller_period.get())+1
      data_L_pos = array('H', points * [0])
      data_R_pos = array('H', points * [0])
      data_L_vel = array('f', points * [0])
      data_R_vel = array('f', points * [0])
      idx = 0
      share_left_effort.put(effort)
      share_right_effort.put(effort)
      share_left_run_dur.put(run_dur)
      share_right_run_dur.put(run_dur)
      state = 2
      yield (state)

    elif state == S2_RUN_AND_COLLECT_DATA:
      # print("T1 S2")
      if (share_left_run_dur.get() == 0 and share_right_run_dur.get() == 0 and 
          share_left_effort.get() == 0 and share_right_effort.get() == 0 and
          queue_left_position.empty() and queue_right_position.empty() and
          queue_left_velocity.empty() and queue_right_velocity.empty()):
        # Run completed and data collected
        state = 3
        yield (state)
      else:  # Motor(s) running and/or data needs processing
        # print(f"idx: {idx}")
        # data_L_pos[idx] = queue_left_position.get()
        # data_R_pos[idx] = queue_right_position.get()
        # data_L_vel[idx] = queue_left_velocity.get()
        # data_R_vel[idx] = queue_right_velocity.get()
        idx += 1
        state = 2
        yield (state)

    elif state == S3_PRINT_RESULTS:
      print("T1 S3")
      for i in range(len(data_L_vel)):
        print(f"{i / points * (run_dur / 1000)}, {data_L_pos[i]}, {data_R_pos[i]}, {data_L_vel[i]}, {data_R_vel[i]}")
      state = 4
      yield (state)

    elif state == S4_HOLD:
      yield (state)

    else:  # Error state
      state = 1
      raise RuntimeError("State machine in task2_motor_controller is in an illegal state. Resetting to state 1.")


def task2_left_motor_controller(shares):
  """!
  Task which controls the left motor using PID velocity control
  @param shares - a tuple of shares and queues for motor control
  """
  # Get references to the share(s) and queue(s)
  share_run_dur, share_left_vel, share_max_eff, share_cur_left_vel, share_wait, share_stop = shares
    
  # State machine variables
  state = 0
  off_time = 0
  S0_CREATE_OBJECTS = 0
  S1_STANDBY = 1
  S2_RUN = 2

  while True:

    if state == S0_CREATE_OBJECTS:
      Motor_Left = Motor(motor_left_enable, motor_left_dir, motor_left_timer, motor_left_channel, motor_left_pwm)
      Encoder_Left = Encoder(encoder_left_timer, encoder_left_A, encoder_left_B)
      # Create PID controller with tuned gains
      PID_Motor_Left = PID(.3, 0.009, 0, 100 / 34.8 / 7.2, 0.44 / 7.2 * 100, integral_error_max=500)
      state = 1
      yield (state)

    elif state == S1_STANDBY:
      if share_wait.get() == 0:
        print("The wait is over!")
        Motor_Left.enable()
        state = 2
        yield (state)
      else:
        state = 1  # Stay in current state
        yield (state)

    elif state == S2_RUN:
      # Update encoder and store data
      Encoder_Left.update()
      current_velocity = Encoder_Left.get_velocity()
      share_cur_left_vel.put(current_velocity)
      # Calculate control effort using PID
      effort = PID_Motor_Left.update(share_left_vel.get(), current_velocity)
      # Constrain effort to valid range
      effort = max(min(effort, 100), -100)
      Motor_Left.set_effort(effort)

      if share_stop.get() == 1:
        Motor_Left.disable()
        Motor_Left.set_effort(0)
        share_run_dur.put(0)
        share_left_vel.put(0)
        # state = 1
        yield (state)
      else:
        state = 2  # Stay in current state
        yield (state)

    else:  # Error state
      state = 1
      print("T2 Error")
      raise RuntimeError("State machine in task2_left_motor_controller is in an illegal state. Resetting to state 1.")


def task3_right_motor_controller(shares):
  """!
  Task which controls the right motor using PID velocity control
  @param shares - a tuple of shares and queues for motor control
  """
  # Get references to the share(s) and queue(s)
  share_run_dur, share_right_vel, share_cur_right_vel, share_wait, share_stop = shares
    
  # State machine variables
  state = 0
  off_time = 0
  S0_CREATE_OBJECTS = 0
  S1_STANDBY = 1
  S2_RUN = 2

  while True:

    if state == S0_CREATE_OBJECTS:
      Motor_Right = Motor(motor_right_enable, motor_right_dir, motor_right_timer, motor_right_channel, motor_right_pwm)
      Encoder_Right = Encoder(encoder_right_timer, encoder_right_A, encoder_right_B)
      # Create PID controller with tuned gains
      PID_Motor_Right = PID(.3, 0.009, 0, 100 / 33.7 / 7.2, 0.49 / 7.2 * 100, integral_error_max=500)
      state = 1
      yield (state)

    elif state == S1_STANDBY:
      if share_wait.get() == 0:
        print("The wait is over! but for the right motor")
        Motor_Right.enable()
        state = 2
        yield (state)
      else:
        state = 1  # Stay in current state
        yield (state)

    elif state == S2_RUN:
      # Update encoder and store data
      Encoder_Right.update()
      current_velocity = Encoder_Right.get_velocity()
      share_cur_right_vel.put(current_velocity)
      # queue_position.put(Encoder_Right.get_position())
      # queue_velocity.put(current_velocity)

      # Calculate control effort using PID
      effort = PID_Motor_Right.update(share_right_vel.get(), current_velocity)
      # Constrain effort to valid range
      effort = max(min(effort, 100), -100)
      Motor_Right.set_effort(effort)

      if share_stop.get() == 1:
        Motor_Right.disable()
        Motor_Right.set_effort(0)
        share_run_dur.put(0)
        share_right_vel.put(0)
        # state = 1
        yield (state)
      else:
        state = 2  # Stay in current state
        yield (state)

    else:  # Error state
      state = 1
      raise RuntimeError("State machine in task3_right_motor_controller is in an illegal state. Resetting to state 1.")
        
def task4_path_follow(shares):
  """!
  Task which reads the current orientation of Romi 
  @param shares A list holding the share(s) used by this task
  """
  # Get references to the share(s)
  share_chasssis_vel, share_left_vel, share_right_vel, share_cur_left_vel, share_cur_right_vel, share_wait, share_mode, share_heading, share_continue = shares
    
  # State machine variables
  state = 0
  S0_INIT = 0
  S1_CALIBRATE = 1
  S2_RUN = 2

  MODE_HEADING = 0
  MODE_IR = 1
  MODE_TURN = 2
  MODE_BACKWARD = 3
  MODE_STOP = 4

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
      # Create IMU object
      imu_obj = IMU()
      # Create angular velocity PID controller
      PID_Omega = PID(.08, 0.00, 0.0)
      # Create Romi object
      chassis = Romi()
      # Create IR object 
      ir_sensor = IR(ir_ctrl, [ir_1, ir_3, ir_5, ir_7])
      bumper = Bump(bump0, bump1, bump2, bump3, bump4, bump5)

      line_follower = LineFollow(ir_sensor)
          
      share_wait.put(1)
      print("IMU object created")
      state = S1_CALIBRATE
      yield(state)
            
    elif state == S1_CALIBRATE:
      # Calibrate IMU sensors, do this until calibration is done
      while imu_obj.startup() == False: 
        for i in range(1000):
          yield(state)
      # Default Cal seems fine
      ir_sensor.default_calibration()
      """           
      # Calibrate IR sensors
      for i in range(5):
        yield(state)
      print("Press the blue button to calibrate black")
      while button.value() == 1:
        yield(state)
      ir_sensor.calibrate_black()
      for i in range(5):
        yield(state)
      print("Press the blue button to calibrate white")
      while button.value() == 1:
        yield(state)
      ir_sensor.calibrate_white()
      for i in range(5):
        yield(state)
      """
      print("Cal done. Press the blue button to start")
      while button.value() == 1:
        yield(state)
      imu_obj.set_mode(IMU.NDOF)
      chassis.zero_heading(imu_obj.read_euler_heading())
      if test_mode == 1:
         chassis.zero_heading(imu_obj.read_euler_heading()+90) 
      state = S2_RUN
      share_wait.put(0)
      share_chassis_vel.put(5*.036)
      for i in range(5):
        yield(state)
      chassis.update_time()
      yield(state)
    
    elif state == S2_RUN:
            
            bumper.update()
            if bumper.get() and bumper_status == 0:
              bumper_status = 1
              chassis.zero_distance()
              mode_index = 0
              share_continue.put(1)
              turn_gen = None
              print("Bumper pressed")

            # Check if we need to switch modes based on distance
            dist = chassis.get_distance()
            print(f"Distance: {dist}")

            if bumper_status == 0:
            # If there's a mode change scheduled and we've reached that distance
              if mode_index < len(mode_schedule) and dist >= mode_schedule[mode_index][0]:
                  # Example: (0.05, "IR") or (0.50, "HEADING", 180)
                  entry = mode_schedule[mode_index]

                  # entry[1] is the mode ("HEADING", "IR", "TURN")
                  # entry[2] might be the angle if present.
                  new_mode = entry[1]
                  print(f"Switching to mode: {new_mode}")
                  share_mode.put(new_mode)

                  # If the mode includes an angle, set share_heading
                  if len(entry) > 2:
                      share_heading.put(entry[2])
                      print(f"Setting heading: {entry[2]}")

                  mode_index += 1
            else:
               if mode_index < len(mode_schedule2) and dist >= mode_schedule2[mode_index][0] and share_continue.get() == 1:
                  # Example: (0.05, "IR") or (0.50, "HEADING", 180)
                  entry = mode_schedule2[mode_index]

                  # entry[1] is the mode ("HEADING", "IR", "TURN")
                  # entry[2] might be the angle if present.
                  new_mode = entry[1]
                  print(f"Switching to mode: {new_mode}")
                  share_mode.put(new_mode)

                  # If the mode includes an angle, set share_heading
                  if len(entry) > 2:
                      share_heading.put(entry[2])
                      print(f"Setting heading: {entry[2]}")

                  mode_index += 1

            # Run whichever mode is currently set.
            current_mode = share_mode.get()

            if current_mode == MODE_IR:
                follow_line(chassis, line_follower, share_cur_left_vel,
                            share_cur_right_vel, share_left_vel,
                            share_right_vel, share_max_vel)

            elif current_mode == MODE_HEADING:
                heading = share_heading.get()  # For instance, 0, 90, 180, etc.
                follow_heading(chassis, imu_obj, PID_Omega, share_chassis_vel,
                               share_cur_left_vel, share_cur_right_vel,
                               share_left_vel, share_right_vel,
                               share_max_vel, heading)

            elif current_mode == MODE_TURN:
                turn_angle = share_heading.get()  # e.g., 90 degrees CW
                if turn_gen is None:
                  # "follow_turn" is a generator; we need to step through it until done.
                  turn_gen = follow_turn(
                      chassis, imu_obj, PID_Omega, share_left_vel,
                      share_right_vel, share_max_vel, turn_angle, share_continue
                  )
                # We'll iterate once on that generator per scheduler cycle until it's done.
                try:
                    next(turn_gen)
                except StopIteration:
                    print("Turn completed probably")
                    # Once the turn is completed, revert mode to something else, e.g. HEADING or IR, or do nothing.
                    turn_gen = None
                    pass

            # Example check for stopping if certain condition is met
            # if dist > 1.0:
            #     share_stop.put(1)
            elif current_mode == MODE_BACKWARD:
                if turn_gen is None:
                  turn_gen = go_backward(chassis, share_left_vel, share_right_vel, share_cur_left_vel.get(), share_cur_right_vel.get(), chassis.get_angle(), share_heading.get(), share_continue)

                try:
                    next(turn_gen)
                except StopIteration:
                    print("Backward completed probably")
                    turn_gen = None
                    # Once the turn is completed, revert mode to something else, e.g. HEADING or IR, or do nothing.
                    pass
            elif current_mode == MODE_STOP:
              raise KeyboardInterrupt
               
            yield state
            
    else:  # Error state
      state = S0_INIT
      raise RuntimeError("Invalid state in task6_path_follow")

def follow_line(chassis, line_follower, cur_left_vel_share, cur_right_vel_share,
                left_vel_share, right_vel_share, max_vel_share):
    """IR-based line following."""
    cur_left_vel = cur_left_vel_share.get()
    cur_right_vel = cur_right_vel_share.get()
    chassis.update(cur_left_vel, cur_right_vel, chassis.get_angle())

    left_vel, right_vel = line_follower.update(max_vel_share)
    left_vel_share.put(left_vel)
    right_vel_share.put(right_vel)

def go_backward(chassis, left_vel_share, right_vel_share, cur_left_vel, cur_right_vel, cur_heading, distance, continue_val):
  chassis.zero_distance()
  continue_val.put(0)
  print("Going backward")
  while True:
    left_vel_share.put(-20)
    right_vel_share.put(-20)
    chassis.update(left_vel_share.get(), right_vel_share.get(), cur_heading)

    error = distance + chassis.get_distance()

    if error < 0.001:
      print("Please please backwards")
      left_vel_share.put(0)
      right_vel_share.put(0)
      chassis.zero_distance()
      chassis.update_time()
      print("Done going backward")
      continue_val.put(1)
      break

    yield 


def follow_heading(chassis, imu_obj, PID_Omega, chassis_vel_share, cur_left_vel_share,
                   cur_right_vel_share, left_vel_share, right_vel_share,
                   max_vel_share, target_heading):
    """Heading hold using IMU and PID."""
    chassis_vel = chassis_vel_share.get()
    current_heading = imu_obj.read_euler_heading()

    cur_left_vel = cur_left_vel_share.get()
    cur_right_vel = cur_right_vel_share.get()
    chassis.update(cur_left_vel, cur_right_vel, current_heading)

    # Calculate heading error
    error = target_heading - chassis.get_angle()
    # Normalize error to [-180, 180]
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
    """Implements a quick turn to a relative angle (e.g., +90 deg)."""
    # We'll do a small loop to achieve the turn. This function is a generator so we can yield.
    # start_heading = imu_obj.read_euler_heading()
    # target_heading = (start_heading + turn_angle) % 360
    target_heading = turn_angle
    print("Target heading: ", target_heading)

    initial_distance = chassis.get_distance()
    print("Initial distance: ", initial_distance)
    continue_val.put(0)
    while True:
        current_heading = imu_obj.read_euler_heading()
        chassis.update_angle(current_heading)
        # Compute heading error
        error = target_heading - chassis.get_angle()
        # Normalize error to [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # If we're close enough to the target, break
        if abs(error) < 3:
            # Stop wheels
            left_vel_share.put(0)
            right_vel_share.put(0)
            chassis.set_distance(initial_distance)
            chassis.update_time() 
            continue_val.put(1)
            print("Turn complete at heading: ", chassis.get_angle())
            break

        chassis_omega = -1 * PID_Omega.update_error(error)
        # Zero linear velocity since we only want to turn
        left_vel, right_vel = chassis.calc_des_wheel_vel(0, chassis_omega, max_vel_share)
        left_vel_share.put(left_vel)
        right_vel_share.put(right_vel)

        # Update chassis state to keep track of overall angle or distance if needed
        
        yield  # Let the scheduler run other tasks

def task5_bumper():
   
  # State machine variables
  state = 0
  S0_INIT = 0
  S1_WAIT_FOR_PRESS = 1
  S2_CRASH_OUT = 2
   
  while True:
    if state == S0_INIT:
      bumper = Bump(bump0, bump1, bump2, bump3, bump4, bump5)
      state = S1_WAIT_FOR_PRESS
      yield(state)
    elif state == S1_WAIT_FOR_PRESS:
      bumper.update()
      if bumper.get():
        print("Bumper pressed")
        state = S2_CRASH_OUT
        yield(state)
      else:
        yield(state)
    elif state == S2_CRASH_OUT:
      raise KeyboardInterrupt
    else:
      state = S0_INIT
      raise RuntimeError("Invalid state in task5_bumper")
        

# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
  print("Testing Lab_0x03 Motor Controllers with Multitasking using cotask.py and task_share.py\r\n"
        "Press Ctrl-C to stop and show diagnostics.")

  # Create a share and a queue to test function and diagnostic printouts
  share_motor_controller_period = task_share.Share('i', thread_protect=False, name="Share_Left_Motor_Controller_Period")
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
  queue_left_position = task_share.Queue('l', q_len, thread_protect=False, overwrite=False, name="Queue_Left_Position")
  queue_left_velocity = task_share.Queue('f', q_len, thread_protect=False, overwrite=False, name="Queue_Left_Velocity")
  queue_right_position = task_share.Queue('l', q_len, thread_protect=False, overwrite=False, name="Queue_Right_Position")
  queue_right_velocity = task_share.Queue('f', q_len, thread_protect=False, overwrite=False, name="Queue_Right_Velocity")

  share_max_vel = task_share.Share('f', thread_protect=False, name="Share_Max_Vel")
  #share_max_vel.put(20)
  share_max_vel.put(40)

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

  # Create the tasks. If trace is enabled for any task, memory will be
  # allocated for state transition tracing, and the application will run out
  task1 = cotask.Task(task1_main, name="Task_1", priority=1, period=30,
                      profile=True, trace=False, 
                      shares=(share_motor_controller_period, share_left_effort, share_right_effort, share_left_run_dur, share_left_run_dur, 
                              queue_left_position, queue_right_position, queue_left_velocity, queue_right_velocity, share_cur_left_vel, share_cur_right_vel))
  task2 = cotask.Task(task2_left_motor_controller, name="Task_2", priority=2, period=share_motor_controller_period.get(), profile=True, trace=False, 
                      shares=(share_left_run_dur, share_left_vel, share_max_eff, share_cur_left_vel, share_wait, share_stop))
  task3 = cotask.Task(task3_right_motor_controller, name="Task_3", priority=2, period=share_motor_controller_period.get(), profile=True, trace=False, 
                    shares=(share_right_run_dur, share_right_vel, share_cur_right_vel, share_wait, share_stop))  
  task4 = cotask.Task(task4_path_follow, name="Task_4", priority=3, period=200, profile=True, trace=False, 
                    shares=(share_chassis_vel, share_left_vel, share_right_vel, share_cur_left_vel, share_cur_right_vel, share_wait, share_mode, share_heading, share_continue))
  task5 = cotask.Task(task5_bumper, name="Task_5", priority=4, period=1000)
  
  #cotask.task_list.append(task1)
  cotask.task_list.append(task2)
  cotask.task_list.append(task3)
  cotask.task_list.append(task4)
  # cotask.task_list.append(task5)

  # Run the memory garbage collector to ensure memory is as defragmented as
  # possible before the real-time scheduler is started
  gc.collect()

  # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
  while True:
    try:
      cotask.task_list.pri_sched()
    except KeyboardInterrupt:
      Motor_Right = Motor(motor_right_enable, motor_right_dir, motor_right_timer, motor_right_channel, motor_right_pwm)
      Motor_Left = Motor(motor_left_enable, motor_left_dir, motor_left_timer, motor_left_channel, motor_left_pwm)
      Motor_Left.disable()
      Motor_Left.set_effort(0)
      Motor_Right.disable()
      Motor_Right.set_effort(0)

      print("Please I'm stopping please")
      break

  # Print a table of task data and a table of shared information data
  print('\n' + str(cotask.task_list))
  print(task_share.show_all())
  print('')