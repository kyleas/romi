import pyb
from pyb import UART

pyb.repl_uart(UART(3))
print("UART 3 set as REPL at 115200 baud")