import math
from time import ticks_ms, ticks_diff  # Use to get dt value in update

class Romi:
  def __init__(self):
    self.width = 0.141 # m
    self.r_w = 0.036 # m
    self.left_velocity = 0
    self.right_velocity = 0
    self.left_position = 0
    self.right_position = 0
    self.prev_time = 0
    self.angle = 0
    self.linear_distance = 0
    self.xpos = 0
    self.ypos = 0

  def zero_heading(self, heading):
    self.heading_offset = heading
    print(f"Heading offset: {self.heading_offset}")

  def update_time(self):
    self.prev_time = ticks_ms()

  def update(self, left_velocity, right_velocity, heading):
    self.left_velocity = left_velocity
    self.right_velocity = right_velocity

    linear_velocity = self.calc_linear_velocity(left_velocity, right_velocity)
    angular_velocity = self.calc_angular_velocity(left_velocity, right_velocity)

    current_time = ticks_ms()
    dt = ticks_diff(current_time, self.prev_time) / 1000
    self.prev_time = current_time

    self.angle = heading - self.heading_offset

    x_vel = linear_velocity * math.cos(self.angle)
    y_vel = linear_velocity * math.sin(self.angle)

    self.xpos += x_vel * dt
    self.ypos += y_vel * dt

    prev_lin_distance = self.linear_distance

    self.linear_distance = self.linear_distance + (linear_velocity * dt)

    if abs(self.linear_distance - prev_lin_distance) > 1:
      print("Reset lin distance")
      self.linear_distance = prev_lin_distance

  def update_angle(self, heading):
    self.angle = heading - self.heading_offset

  def calc_des_wheel_vel(self, linear, angular, MAX_VEL):
    """
    Input linear velocity in m/s and angular velocity in rad/s
    Output left and right wheel velocities in rpm
    """
    max_vel = MAX_VEL.get()
    left = (linear / self.r_w - angular * self.width / 2 / self.r_w) * 9.5492968
    right = (linear / self.r_w + angular * self.width / 2 / self.r_w) * 9.5492968
    
    print(f"linear: {linear/self.r_w}, angular: {angular * self.width / 2 / self.r_w}")
    # Ensure the velocities do not exceed the maximum velocity
    #left = max(min(left, max_vel), -max_vel)
    #right = max(min(right, max_vel), -max_vel)
    
    return (left, right)
  
  # Input left and right wheel velocities in rpm
  # Output linear velocity in m/s
  def calc_linear_velocity(self, left, right):
    return (left + right) * self.r_w / 2 / 9.5492968
  
  # Input left and right wheel velocities in rpm
  # Output angular velocity in rad/s
  def calc_angular_velocity(self, left, right):
    return (right - left) / 9.5492968 / self.width * self.r_w
  
  def get_distance(self):
    return self.linear_distance
  
  def zero_distance(self):
    self.linear_distance = 0
    self.prev_time = ticks_ms()

  def get_angle(self):
    return self.angle
  
  def set_distance(self, distance):
    self.linear_distance = distance