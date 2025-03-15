import pyb
from pyb import Pin
from array import array
from lib2.pin_definitions import * 

class Bump:
  """
  A Romi bumper pair interface encapsulated in a Python class.
  """

  def __init__(self, BUMP0, BUMP1, BUMP2, BUMP3, BUMP4, BUMP5):
    """
    Initializes an Bump object
    """    
    self.BUMP0_pin = Pin(BUMP0, Pin.IN, Pin.PULL_UP)
    self.BUMP1_pin = Pin(BUMP1, Pin.IN, Pin.PULL_UP)
    self.BUMP2_pin = Pin(BUMP2, Pin.IN, Pin.PULL_UP)
    self.BUMP3_pin = Pin(BUMP3, Pin.IN, Pin.PULL_UP)
    self.BUMP4_pin = Pin(BUMP4, Pin.IN, Pin.PULL_UP)
    self.BUMP5_pin = Pin(BUMP5, Pin.IN, Pin.PULL_UP)

    self.BUMP_Array = array('i', [0, 0, 0, 0, 0, 0])

  def update(self):
    """
    Updates the data in the array values
    """
    self.BUMP_Array[5] = self.BUMP0_pin.value()
    self.BUMP_Array[4] = self.BUMP1_pin.value()
    self.BUMP_Array[3] = self.BUMP2_pin.value()
    self.BUMP_Array[2] = self.BUMP3_pin.value()
    self.BUMP_Array[1] = self.BUMP4_pin.value()
    self.BUMP_Array[0] = self.BUMP5_pin.value()

  def get(self):
    """
    Returns the array values
    """
    for i in range(6):
      if self.BUMP_Array[i] == 0:
        return True
    return False
    #return self.BUMP_Array

if __name__ == "__main__":
  """
  Test code for the Bump class
  """
  bump = Bump(bump0, bump1, bump2, bump3, bump4, bump5)
  while True:
    bump.update()
    print(bump.get())
    pyb.delay(1000)