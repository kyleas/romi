"""!
@file ir.py
@brief IR sensor reading and calibration module for line detection.
@details
  Creates an IR class that can read four IR sensors via ADC, calibrate their
  black/white levels, and compute a normalized position centroid for line following.
"""

import pyb
from pyb import Pin, Timer, ADC
from array import array
from lib2.pin_definitions import *
from time import sleep_ms

class IR:
    """!
    @class IR
    @brief IR sensor interface class for line following on the ROMI chassis.
    @details
      Uses four analog inputs to measure reflectance. The data is scaled 
      between min and max values determined by calibration.
    """

    def __init__(self, ODD, PINS):
        """!
        @brief Initializes the IR sensor array.
        @param ODD Pin controlling the IR LED power (ODD_pin).
        @param PINS List of 4 pin identifiers used for IR sensor ADC reads.
        @exception ValueError If PINS does not have exactly 4 elements.
        """
        self.ODD_pin = Pin(ODD, mode=Pin.OUT_PP)
        self.ODD_pin.high()

        if len(PINS) != 4:
            raise ValueError("PINS list must have exactly 4 elements")

        PIN1_pin = Pin(PINS[0], mode=Pin.IN)
        PIN3_pin = Pin(PINS[1], mode=Pin.IN)
        PIN5_pin = Pin(PINS[2], mode=Pin.IN)
        PIN7_pin = Pin(PINS[3], mode=Pin.IN)

        self.IR1 = pyb.ADC(PIN1_pin)
        self.IR3 = pyb.ADC(PIN3_pin)
        self.IR5 = pyb.ADC(PIN5_pin)
        self.IR7 = pyb.ADC(PIN7_pin)

        self.values = array('f', len(PINS) * [0])

        # Default calibrations
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
        """!
        @brief Updates the IR sensor readings, scaling them between 0 and 1.
        @details
          Reads each ADC, subtracts the min calibration, and divides by 
          the (max - min) range.
        """
        self.values[0] = (self.IR1.read() - self.min1) / (self.max1 - self.min1)
        self.values[1] = (self.IR3.read() - self.min3) / (self.max3 - self.min3)
        self.values[2] = (self.IR5.read() - self.min5) / (self.max5 - self.min5)
        self.values[3] = (self.IR7.read() - self.min7) / (self.max7 - self.min7)

    def calc_centroid(self):
        """!
        @brief Computes a centroid-based position indicator from -1 to +1.
        @details
          The IR sensors are conceptually at positions [0..3]. Weighted by readings,
          the centroid is computed. Then scaled to [-1..1].
        
        @return A float from -1 to +1 representing the line's position under the sensors.
        """
        self.update()
        sum_values = 0
        moment = 0
        for i in range(self.pins):
            moment += self.values[i] * i
            sum_values += self.values[i]

        if sum_values > 0:
            return moment / sum_values / ((self.pins - 1) / 2) - 1
        return 0

    def calibrate_black(self):
        """!
        @brief Calibrates sensor readings for the 'black' (high reflection) case.
        @details
          Updates the max calibration values based on the current ADC readings.
        """
        self.max1 = self.IR1.read()
        self.max3 = self.IR3.read()
        self.max5 = self.IR5.read()
        self.max7 = self.IR7.read()
        max_vals = [self.max1, self.max3, self.max5, self.max7]
        print(f"Black: {max_vals}")

    def calibrate_white(self):
        """!
        @brief Calibrates sensor readings for the 'white' (low reflection) case.
        @details
          Updates the min calibration values based on the current ADC readings.
        """
        self.min1 = self.IR1.read()
        self.min3 = self.IR3.read()
        self.min5 = self.IR5.read()
        self.min7 = self.IR7.read()
        min_vals = [self.min1, self.min3, self.min5, self.min7]
        print(f"White: {min_vals}")

    def default_calibration(self):
        """!
        @brief Loads default calibration values known to be typical for the ROMI's IR sensors.
        """
        self.min1 = 261
        self.min3 = 251
        self.min5 = 251
        self.min7 = 262

        self.max1 = 3759
        self.max3 = 3656
        self.max5 = 3403
        self.max7 = 3609


if __name__ == "__main__":
    """!
    @brief Example usage of IR class for manual calibration.
    """
    my_IR = IR(ir_ctrl, [ir_1, ir_3, ir_5, ir_7])
    input("Hit enter to calibrate black")
    my_IR.calibrate_black()
    input("Hit enter to calibrate white")
    my_IR.calibrate_white()
    sleep_ms(5000)
    while True:
        my_IR.update()
        print(f"1: {my_IR.values[0]}, 2: {my_IR.values[1]}, 3: {my_IR.values[2]}, 4: {my_IR.values[3]}")
        centroid = my_IR.calc_centroid()
        print(f"Centroid: {centroid}")
        sleep_ms(500)
