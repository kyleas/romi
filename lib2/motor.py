"""!
@file motor.py
@brief Motor driver interface for the ROMI chassis.
@details 
  This module provides a class to control a DC motor via PWM signals and a 
  direction pin. It works with drivers like the DRV8838 for Pololu's ROMI 
  chassis, featuring functions to set motor effort, enable, and disable the driver.
"""

from pyb import Pin, Timer

class Motor:
    """!
    @class Motor
    @brief A motor driver interface encapsulated in a Python class.
    @details 
      This class creates a motor driver object controlling PWM and direction pins 
      for a single motor. The driver uses separate PWM and direction lines, e.g., 
      DRV8838 modules on a ROMI chassis.
    """

    def __init__(self, nSLP, DIR, TIM, CH, PWM):
        """!
        @brief Initializes a Motor object.
        @param nSLP Pin identifier for the motor's sleep (enable) pin.
        @param DIR Pin identifier for the motor's direction pin.
        @param TIM Timer number used for generating PWM signals.
        @param CH The timer channel for PWM output.
        @param PWM The pin to which PWM output is assigned.
        """
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP)
        self.TIM = Timer(TIM, freq=20000)
        self.PWM_pin = self.TIM.channel(CH, Timer.PWM, pin=PWM)

    def set_effort(self, effort):
        """!
        @brief Sets the motor effort (speed and direction).
        @details 
          The effort value must be between -100 and 100. Negative values 
          cause the motor to spin in reverse, while positive values spin forward.
        
        @exception ValueError If effort is outside the range -100 to 100.
        @param effort Motor speed setting, from -100 (full reverse) 
                      to +100 (full forward).
        """
        if effort < -100 or effort > 100:
            raise ValueError("Effort value must be between -100 and 100")
        elif effort < 0:
            self.DIR_pin.high()   # Reverse
        else:
            self.DIR_pin.low()    # Forward

        self.PWM_pin.pulse_width_percent(abs(effort))

    def enable(self):
        """!
        @brief Enables the motor driver by taking it out of sleep mode.
        @details 
          Sets the nSLP (not sleep) pin high, activating the motor driver 
          to brake mode with 0% duty cycle initially.
        """
        self.nSLP_pin.high()
        self.PWM_pin.pulse_width_percent(0)

    def disable(self):
        """!
        @brief Disables the motor driver by putting it into sleep mode.
        @details 
          Sets the nSLP pin low, halting motor activity.
        """
        self.nSLP_pin.low()
