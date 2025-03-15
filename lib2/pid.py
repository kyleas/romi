from time import ticks_us, ticks_diff, ticks_ms  # Use to get dt value in update
from pyb import Pin, Timer
import cqueue as cqueue
import time

class PID:
  """
  A PID loop encapsulated in a Python class
  """

  def __init__(self, Kp, Ki, Kd, feedforward_gain=0, feedforward_offset=0, integral_error_max=500):
    """
    Initializes gains and integral error
    """
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.ff_gain = feedforward_gain
    self.ff_offset = feedforward_offset
    self.integral_error = 0 
    self.prev_error = 0
    self.prev_time = 0
    self.integral_error_max = integral_error_max

  def update(self, desire, actual):
    """
    Generates a reponse
    """
    dt = ticks_diff(ticks_ms(), self.prev_time) / 1000
    self.prev_time = ticks_ms()

    error = desire - actual
    self.integral_error += error * dt
    if self.integral_error > self.integral_error_max:
      self.integral_error = self.integral_error_max
    elif self.integral_error < -self.integral_error_max:
      self.integral_error = -self.integral_error_max
    derivative = error - self.prev_error
    self.prev_error = error
    dir = 1 if desire > 0 else -1 if desire < 0 else 0
    return self.Kp * error + self.integral_error * self.Ki + self.Kd * derivative + self.ff_gain * desire + (self.ff_offset * dir)
  
  def update_error(self, error):
    """
    Generates a reponse
    """
    dt = ticks_diff(ticks_ms(), self.prev_time) / 1000
    self.prev_time = ticks_ms()

    self.integral_error += error * dt
    derivative = error - self.prev_error
    self.prev_error = error
    return self.Kp * error + self.integral_error * self.Ki + self.Kd * derivative 
