from pyb import Pin, Timer

class Motor:
  """
  A motor driver interface encapsulated in a Pytohn class. Works with
  motor drivers using separate PWM and direction inputs such as the DRV8838
  drivers present on the ROMI chassis from Polulu.
  """

  def __init__(self, nSLP, DIR, TIM, CH, PWM):
    """
    Initializes a Motor object
    """
    self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value = 0)
    self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP)
    self.TIM = Timer(TIM, freq=20000)
    self.PWM_pin = self.TIM.channel(CH, Timer.PWM, pin=PWM)

  def set_effort(self, effort):
    """
    Sets the present effort requested from the motor based on an input value
    between -100 and 100
    """
    if effort < -100 or effort > 100:
      raise ValueError("Effort value must be between -100 and 100")
    elif effort < 0:
      self.DIR_pin.high()  # Reverse
    else:
      self.DIR_pin.low()  # Forward
      
    self.PWM_pin.pulse_width_percent(abs(effort))  # NOTE: if effort is 0, then PWM is set to 0

  def enable(self):
    """
    Enables the motor driver by taking it out of sleep mode into brake mode
    """
    self.nSLP_pin.high()  # Brake mode
    self.PWM_pin.pulse_width_percent(0)  # 0% duty cycle

  def disable(self):
    """
    Disables the motor driver by putting it into sleep mode
    """
    self.nSLP_pin.low()  # Sleep mode
