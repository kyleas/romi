"""!
@file encoder.py
@brief A quadrature encoder interface class for measuring position and velocity.
@details
  Provides an update method to track position changes and compute velocity in 
  RPM using a short history of positions and timestamps.
"""

from time import ticks_us, ticks_diff
from pyb import Pin, Timer
import cqueue as cqueue

class Encoder:
    """!
    @class Encoder
    @brief A quadrature encoder decoding interface.
    @details
      Uses a hardware timer in ENC_AB mode to count position. Keeps a small queue
      of past positions/timestamps to estimate velocity.
    """

    def __init__(self, timer, pin_a, pin_b):
        """!
        @brief Initializes the encoder with the given timer and pins.
        @param timer The timer number configured for quadrature decoding.
        @param pin_a The pin for channel A of the quadrature signal.
        @param pin_b The pin for channel B of the quadrature signal.
        """
        self.position = 0
        self.prev_count = 0
        self.delta = 0
        self.dt = 0
        self.prev_time = 0

        self.tim = Timer(timer, period=0xFFFF, prescaler=0)
        self.tim.channel(1, pin=pin_a, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=pin_b, mode=Timer.ENC_AB)

        # Deques to store position/time history for velocity calculation
        self.position_history = cqueue.IntQueue(5)
        self.time_history = cqueue.IntQueue(5)

        self.update()

    def update(self):
        """!
        @brief Updates internal position/velocity calculations with latest timer counts.
        @details
          Reads the hardware timer, calculates delta from last reading, and 
          adjusts for rollovers. Stores position and timestamp in FIFO queues.
        """
        self.delta = self.tim.counter() - self.prev_count
        if self.delta > ((0xFFFF + 1) / 2):
            self.delta -= (0xFFFF + 1)
        elif self.delta < -((0xFFFF + 1) / 2):
            self.delta += (0xFFFF + 1)

        self.position = self.prev_count + self.delta
        self.prev_count = self.tim.counter()

        self.dt = ticks_diff(ticks_us(), self.prev_time)
        self.prev_time = ticks_us()

        self.position_history.put(self.position)
        self.time_history.put(self.prev_time)

    def get_position(self):
        """!
        @brief Returns the most recently computed position.
        @return The encoder's current position in counts (relative to zero()).
        """
        return self.position

    def get_velocity(self):
        """!
        @brief Computes velocity in RPM based on historical data.
        @details
          Uses the difference in position between now and 5 updates ago 
          (if the queue is full) and the time difference. 
          Formula: 
            velocity (RPM) = (delta_counts * 60_000_000) / (time_delta_us * 1440).
        
        @return The velocity in RPM or 0.0 if there's insufficient data.
        """
        if self.position_history.full():
            position_5_updates_ago = self.position_history.get()
            time_5_updates_ago = self.time_history.get()
            position_delta = float(self.position - position_5_updates_ago)

            current_time = self.prev_time
            time_delta = ticks_diff(current_time, time_5_updates_ago)

            if time_delta > 0:
                # Convert from counts/time to RPM: 
                #   (counts) / (us) * (60_000_000 us/min) * (1 rev / 1440 counts)
                velocity = (position_delta * 60_000_000.0) / (time_delta * 1440.0)
                return velocity

        return 0.0

    def zero(self):
        """!
        @brief Resets the encoder's position to zero.
        @details Clears the position history so velocity computations restart from zero.
        """
        self.position = 0
        self.prev_count = self.tim.counter()
        self.delta = 0
        self.dt = 0
        self.position_history.clear()
        self.time_history.clear()
