import pyb
from pyb import Pin, Timer, ADC
from array import array
from lib2.pin_definitions import * 
from time import sleep_ms

class IR:
  """
  A motor driver interface encapsulated in a Pytohn class. Works with
  motor drivers using separate PWM and direction inputs such as the DRV8838
  drivers present on the ROMI chassis from Polulu.
  """

  def __init__(self, ODD, PINS):
    """
    Initializes an IR object
    """    
    self.ODD_pin = Pin(ODD, mode=Pin.OUT_PP)
    self.ODD_pin.high()
    
    if len(PINS) != 4:
        raise ValueError("PINS list must have exactly 4 elements")
    
    PIN1_pin = Pin(PINS[0], mode=Pin.IN)
    PIN3_pin = Pin(PINS[1], mode=Pin.IN)
    PIN5_pin = Pin(PINS[2], mode=Pin.IN)
    PIN7_pin = Pin(PINS[3], mode=Pin.IN)

    self.IR1 = pyb.ADC(PIN1_pin)  # ADC has max value 4095
    self.IR3 = pyb.ADC(PIN3_pin)
    self.IR5 = pyb.ADC(PIN5_pin)
    self.IR7 = pyb.ADC(PIN7_pin)

    self.values = array('f', len(PINS) * [0])
    
    self.min1 = 0
    self.min3 = 0
    self.min5 = 0
    self.min7 = 0
    
    self.max1 = 5.5
    self.max3 = 5.5
    self.max5 = 5.5
    self.max7 = 5.5

    self.pins = len(PINS)

  def update(self):
    """
    Updates the data in the array values
    """
    self.values[0] = (self.IR1.read() - self.min1) / (self.max1 - self.min1)
    self.values[1] = (self.IR3.read() - self.min3) / (self.max3 - self.min3)
    self.values[2] = (self.IR5.read() - self.min5) / (self.max5 - self.min5)
    self.values[3] = (self.IR7.read() - self.min7) / (self.max7 - self.min7)

  def calc_centroid(self):
    """
    Enables the motor driver by taking it out of sleep mode into brake mode
    """
    self.update()
    sum_values = 0 
    moment = 0
    for i in range(self.pins):
      moment += self.values[i] * i 
      sum_values += self.values[i]

    # Returns a value from -1 to 1 depending on where the centroid of the line is
    if sum_values > 0:
      return moment / sum_values / ((self.pins-1) / 2) - 1
    return 0

  def calibrate_black(self):
    """
    Disables the motor driver by putting it into sleep mode
    """
    self.max1 = self.IR1.read()
    self.max3 = self.IR3.read()
    self.max5 = self.IR5.read()
    self.max7 = self.IR7.read()
    max_vals = [self.max1, self.max3, self.max5, self.max7]
    print(f"Black: {max_vals}")

  def calibrate_white(self):
    """
    Disables the motor driver by putting it into sleep mode
    """
    self.min1 = self.IR1.read()
    self.min3 = self.IR3.read()
    self.min5 = self.IR5.read()
    self.min7 = self.IR7.read()
    min_vals = [self.min1, self.min3, self.min5, self.min7]
    print(f"White: {min_vals}")

  def default_calibration(self):
    self.min1 = 261
    self.min3 = 251
    self.min5 = 251
    self.min7 = 262

    self.max1 = 3759
    self.max3 = 3656
    self.max5 = 3403
    self.max7 = 3609
    

if __name__ == "__main__":
   my_IR = IR(ir_ctrl, [ir_1, ir_3, ir_5, ir_7])
   input("Hit enter to calibrate black")
   my_IR.calibrate_black()
  #  print(my_IR.max_val)
   input("Hit enter to calibrate white")
   my_IR.calibrate_white()
  #  print(my_IR.min_val)
  #  print(f"HIGH: {my_IR.max_val}, LOW: {my_IR.min_val}")
   sleep_ms(5000)
   while True:
      my_IR.update()
      print(f"1: {my_IR.values[0]}, 2: {my_IR.values[1]}, 3: {my_IR.values[2]}, 4: {my_IR.values[3]}")
      centroid = my_IR.calc_centroid()
      print(f"Centroid: {centroid}")
      sleep_ms(500)
      