from time import ticks_us, ticks_diff  # Use to get dt value in update
from pyb import Pin, Timer
import cqueue as cqueue

class Encoder:
  """
  A quadrature encoder decoding interface encapsulated in a Python class
  """

  def __init__(self, timer, pin_a, pin_b):
    """
    Initializes an Encoder object
    """
    self.position = 0         # Total accumulated position of the encoder
    self.prev_count = 0       # Counter value from the most recent update
    self.delta = 0            # Change in count between last two updates
    self.dt = 0               # Amount of time between last two updates
    self.prev_time = 0        # Time of last update

    # Initialize the timer and encoder channels
    self.tim = Timer(timer, period=0xFFFF, prescaler=0)
    self.tim.channel(1, pin=pin_a, mode=Timer.ENC_AB)
    self.tim.channel(2, pin=pin_b, mode=Timer.ENC_AB)
    
    # Initialize a deque to store positions along with their timestamps (maxlen=5)
    self.position_history = cqueue.IntQueue(5)
    self.time_history = cqueue.IntQueue(5)
    
    self.update()

  def update(self):
    """
    Runs one update step on the encoder's timer counter to keep
    track of the change in count and check for counter reload
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

    # Add the current position and timestamp to the history queue
    self.position_history.put(self.position)
    self.time_history.put(self.prev_time)

  def get_position(self):
    """
    Returns the change in position (delta) between the current position and the position 
    from 5 updates ago.
    """
    return self.position

  def get_velocity(self):
    """
    Returns the velocity in RPM based on the change in position between the current position
    and the position from 5 updates ago, as well as the time between those updates.
    """
    if self.position_history.full():
      # Get the position from 5 updates ago and the current position
      position_5_updates_ago = self.position_history.get()
      time_5_updates_ago = self.time_history.get()
      position_delta = float(self.position - position_5_updates_ago)
      
      # Get the current time and calculate the exact time delta
      current_time = self.prev_time
      # time_delta = current_time - time_5_updates_ago
      time_delta = ticks_diff(current_time, time_5_updates_ago)

      # Calculate velocity: position change divided by time change
      if time_delta > 0:
        # Convert to RPM:
        # (counts) / (microseconds) * (60_000_000 microseconds/minute) * (1 revolution/1440 counts)
        velocity = (position_delta * 60_000_000.0) / (time_delta * 1440.0)
        return velocity
    return 0.0

  def zero(self):
    """
    Sets the present encoder position to zero and causes future updates
    to measure with respect to the new zero position.
    """
    self.position = 0
    self.prev_count = self.tim.counter()
    self.delta = 0
    self.dt = 0
    self.position_history.clear()  # Clear position history on reset
