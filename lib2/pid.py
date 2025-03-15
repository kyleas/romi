"""!
@file pid.py
@brief PID controller class for motor or system control.
@details
  Implements a simple PID algorithm for controlling velocity, position, or
  other feedback-based systems. Includes optional feedforward terms.
"""

from time import ticks_us, ticks_diff, ticks_ms
from pyb import Pin, Timer
import cqueue as cqueue
import time

class PID:
    """!
    @class PID
    @brief A PID loop encapsulated in a Python class.
    @details
      Includes proportional, integral, and derivative terms, as well as optional
      feedforward gain and offset. Integrator windup is limited by integral_error_max.
    """

    def __init__(self, Kp, Ki, Kd, feedforward_gain=0, feedforward_offset=0, integral_error_max=500):
        """!
        @brief Initializes PID gains and integral error parameters.
        @param Kp Proportional gain.
        @param Ki Integral gain.
        @param Kd Derivative gain.
        @param feedforward_gain Optional feedforward gain term applied to the desired setpoint.
        @param feedforward_offset Optional offset applied to the feedforward term based on direction.
        @param integral_error_max Maximum absolute value for the integrator to prevent windup.
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
        """!
        @brief Calculates a control effort using a PID approach (plus feedforward).
        @details
          Uses the difference between desired and actual values (error) to compute the 
          PID output. Limits integral error to prevent excessive windup.
        @param desire The target or desired value.
        @param actual The current or measured value.
        @return The control effort as a float (could be motor PWM, for example).
        """
        dt = ticks_diff(ticks_ms(), self.prev_time) / 1000
        self.prev_time = ticks_ms()

        error = desire - actual
        self.integral_error += error * dt
        # Bound integrator
        if self.integral_error > self.integral_error_max:
            self.integral_error = self.integral_error_max
        elif self.integral_error < -self.integral_error_max:
            self.integral_error = -self.integral_error_max

        derivative = error - self.prev_error
        self.prev_error = error

        # Determine direction for feedforward offset
        dir = 1 if desire > 0 else -1 if desire < 0 else 0

        return (self.Kp * error
                + self.integral_error * self.Ki
                + self.Kd * derivative
                + self.ff_gain * desire
                + (self.ff_offset * dir))

    def update_error(self, error):
        """!
        @brief Calculates a control effort given a direct error (without reading actual).
        @details
          Similar to update(), but uses a provided error directly. 
          Useful for situations where the user has precomputed the error externally.
        @param error The difference between the desired value and measured value.
        @return The control effort as a float.
        """
        dt = ticks_diff(ticks_ms(), self.prev_time) / 1000
        self.prev_time = ticks_ms()

        self.integral_error += error * dt
        derivative = error - self.prev_error
        self.prev_error = error

        return (self.Kp * error
                + self.integral_error * self.Ki
                + self.Kd * derivative)
