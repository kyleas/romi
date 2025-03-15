"""!
@file bump.py
Provides a Bump class that reads six bumper (tactile switch) pins on the
ROMI chassis. It can update the state of each pin and detect if any bumper 
is pressed (active low).
@brief Bumper switch interface for the ROMI robot.
@details
  Provides a Bump class that reads six bumper (tactile switch) pins on the
  ROMI chassis. It can update the state of each pin and detect if any bumper 
  is pressed (active low).
"""

import pyb
from pyb import Pin
from array import array
from lib2.pin_definitions import *

class Bump:
    """!
    @class Bump
    @brief Encapsulates ROMI bumper switch reading.
    @details
      The ROMI can have up to six bump/tactile switches. Each is read as 
      a digital input with pull-up, returning 0 if pressed, 1 if released.
    """

    def __init__(self, BUMP0, BUMP1, BUMP2, BUMP3, BUMP4, BUMP5):
        """!
        @brief Constructs a Bump object with pin references for six switches.
        @param BUMP0 Pin identifier for bumper 0.
        @param BUMP1 Pin identifier for bumper 1.
        @param BUMP2 Pin identifier for bumper 2.
        @param BUMP3 Pin identifier for bumper 3.
        @param BUMP4 Pin identifier for bumper 4.
        @param BUMP5 Pin identifier for bumper 5.
        """
        self.BUMP0_pin = Pin(BUMP0, Pin.IN, Pin.PULL_UP)
        self.BUMP1_pin = Pin(BUMP1, Pin.IN, Pin.PULL_UP)
        self.BUMP2_pin = Pin(BUMP2, Pin.IN, Pin.PULL_UP)
        self.BUMP3_pin = Pin(BUMP3, Pin.IN, Pin.PULL_UP)
        self.BUMP4_pin = Pin(BUMP4, Pin.IN, Pin.PULL_UP)
        self.BUMP5_pin = Pin(BUMP5, Pin.IN, Pin.PULL_UP)

        self.BUMP_Array = array('i', [0, 0, 0, 0, 0, 0])

    def update(self):
        """!
        @brief Updates the bumper readings (populates BUMP_Array).
        @details
          The array is stored in an order that correlates with the physical 
          layout. A 0 value indicates pressed (pin driven low).
        """
        self.BUMP_Array[5] = self.BUMP0_pin.value()
        self.BUMP_Array[4] = self.BUMP1_pin.value()
        self.BUMP_Array[3] = self.BUMP2_pin.value()
        self.BUMP_Array[2] = self.BUMP3_pin.value()
        self.BUMP_Array[1] = self.BUMP4_pin.value()
        self.BUMP_Array[0] = self.BUMP5_pin.value()

    def get(self):
        """!
        @brief Returns True if any bumper is pressed, False otherwise.
        @details
          Iterates through BUMP_Array looking for a 0 (pressed).
        @return True if any switch is pressed, False if none are pressed.
        """
        for val in self.BUMP_Array:
            if val == 0:
                return True
        return False


if __name__ == "__main__":
    """!
    @brief Test code for the Bump class.
    @details
      Continuously reads bumper pins and prints the result every second.
    """
    bump = Bump(bump0, bump1, bump2, bump3, bump4, bump5)
    while True:
        bump.update()
        print(bump.get())
        pyb.delay(1000)
