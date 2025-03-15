import pyb
from pyb import UART

pyb.repl_uart(UART(3, 115200))
print("UART 3 set as REPL at baud 115200")