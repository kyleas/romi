import pyb
from time import sleep_ms
from lib.pin_definitions import blue_button
# Create button object (assuming blue_button is defined in pin_definitions)
# If not defined, typically it's 'PC13' or 'SW1' on PyBoard
button = pyb.Pin(blue_button, pyb.Pin.IN, pyb.Pin.PULL_UP)

while True:
    # Check if button is pressed (returns 0 when pressed, 1 when not pressed)
    if button.value() == 0:
        print("Button pressed!")
        # Small delay to avoid multiple prints from one press
        sleep_ms(200)
