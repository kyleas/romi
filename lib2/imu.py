import pyb
from pyb import Pin, Timer, ADC, I2C
from array import array
from lib2.pin_definitions import * 
from time import sleep_ms
import struct
import os
import time

class IMU:
  """
  A motor driver interface encapsulated in a Pytohn class. Works with
  motor drivers using separate PWM and direction inputs such as the DRV8838
  drivers present on the ROMI chassis from Polulu.
  """
  
  # Operative Mode Variables
  REG_OPR_MODE = 0x3D     # Register Address
  # Operating Mode Values: page 21
  CONFIG = 0x00           # CONFIG MODE
  IMU = 0x08              # Fusion Mode: IMU
  COMPASS = 0x09          # Fusion Mode: COMPASS
  M4G = 0x0A              # Fusion Mode: M4G
  NDOF_FMC_OFF = 0x0B     # Fusion Mode: NDOF_FMC_OFF
  NDOF = 0x0C             # Fusion Mode: NDOF

  # Calibration Variable
  REG_CALIB_STAT = 0x35   # page 67
  REG_CAL_DATA_START = 0x55  # Start of calibration coefficient registers

  # Euler Variables
  REG_EULER = 0x1A        # Euler begins at 1A ends at 1F

  # Gyro Variables
  REG_GYRO = 0x14

  # Magnetometer Variables
  REG_MAG = 0x0E
  
  def __init__(self, id=2, addr=0x28):
    """
    Initializes an IMU object
    """    
    self.IMU = I2C(id, I2C.CONTROLLER)
    self.addr = addr
    self.accel_offset = [0, 0, 0]
    self.mag_offset = [0, 0, 0]
    self.gyr_offset = [0, 0, 0]
    self.accel_rad = 0
    self.mag_rad = 0
    self.mode = IMU.NDOF  # Default mode is NDOF
    self.file = True

  def startup(self):
    status = False
    if self.file == True and IMU.file_exists('calibrations.bin'):
      print("Calibration file found.")
      # Check if the binary calibration file exists
      status = self.write_calibration_coeff()

    if self.file == True and status == False:
      # First run and no valid calibration found
      # Set to CONFIG mode to calibrate 
      self.set_mode(IMU.NDOF)

    self.file = False
    
    if status == False:
      print("Calibration file not legit/doesn't exist. Continue calibration...")
      # Wait for the calibration to complete
      if self.retrieve_calibration_coeff() == -1:
        return False 
      print("Calibration completed.")

    print("Startup complete.")
    return True
  
  def file_exists(filename):
    try:
      # Attempt to open the file to check if it exists
      with open(filename, 'rb') as f:
        print("file present")
        return True
    except OSError:
      # If an error occurs, the file does not exist
      print("file not present")
      return False

  def read_file(filename):
    try:
        with open(filename, 'rb') as f:  # Open the file in binary read mode
            data = f.read()  # Read the entire binary content of the file
        print("File content (in bytes):", data)  # Print the content in byte format
        return data  # Return the binary data
    except OSError:
        print(f"Error: The file {filename} does not exist or cannot be accessed.")
        return None

  def set_mode(self, mode):
    # sets IMU to mode specified in first parameter
    self.mode = mode
    self.IMU.mem_write(mode, self.addr, IMU.REG_OPR_MODE)
    sleep_ms(100)

  def calibration_status(self):
    # page 67
    buf = bytearray((0 for n in range(1)))
    self.IMU.mem_read(buf, self.addr, IMU.REG_CALIB_STAT)
    bits = [((b >> i) & 1) for b in buf for i in range(7, -1, -1)]
    print(bits)
 
    compare = bytearray((0 for n in range(1)))
    compare[0] = 0xff
    if (buf == compare):
      return 1
    return 0

  def retrieve_calibration_coeff(self):
    # Ensure calibration status is 1 before proceeding
    if self.calibration_status() == 0:
      print("Calibration not completed. Exiting.")
      return -1
    self.set_mode(IMU.CONFIG)
    offset = bytearray(22)  # 22 bytes expected for calibration data
    self.IMU.mem_read(offset, self.addr, IMU.REG_CAL_DATA_START)
    try:
      # Unpack the data into individual values for accel, mag, and gyr offsets
      accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z, accel_rad, mag_rad = struct.unpack("<hhhhhhhhhhh", offset)
      # Store the offsets in the instance
      self.accel_offset = [accel_x, accel_y, accel_z]
      self.mag_offset = [mag_x, mag_y, mag_z]
      self.gyr_offset = [gyr_x, gyr_y, gyr_z]
      self.accel_rad = accel_rad
      self.mag_rad = mag_rad
      # Calibration data as a tuple needed to write to file
      calibration_data = (accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z, accel_rad, mag_rad)
      with open('calibrations.bin', 'wb') as f:  # Open in binary write mode
        f.write(struct.pack("<hhhhhhhhhhh", *calibration_data))
        print("Calibration data written to binary file.")
      self.set_mode(self.mode)
    except struct.error as e:
      print(f"Error unpacking calibration data: {e}")
    except IOError as e:
      print(f"Error writing to binary file: {e}")

    # For testing read back from file
    try:
      with open('calibrations.bin', 'rb') as f:
        read_data = f.read(22)  # 22 bytes (because we're storing 10 2-byte integers)
        unpacked_data = struct.unpack("<hhhhhhhhhhh", read_data)
        print(f"Data read from binary file: {unpacked_data}")
    except IOError as e:
      print(f"Error reading from binary file: {e}")
  
  def write_calibration_coeff(self, filename='calibrations.bin'):
    success = False
    self.set_mode(IMU.CONFIG)
    try:
      with open(filename, 'rb') as f:  # Open the file in binary read mode
        data = f.read()  # Read the entire binary content of the file
      print("File content (in bytes):", data)  # Print the content in byte format
      if len(data) != 22:
        print("Error: The calibration data must be 22 bytes long.")
        return False
      decoded_data = struct.unpack("<hhhhhhhhhhh", data)
      print("Decoded Data:", decoded_data)
      self.IMU.mem_write(data, self.addr, IMU.REG_CAL_DATA_START)
      success = True
    except OSError:
      print(f"Error: The file {filename} does not exist or cannot be accessed.")
      raise
    except struct.error as e:
      print(f"Error unpacking data: {e}")
      raise
    finally:
      self.set_mode(self.mode)
      return success

  def read_euler(self):
    euler = bytearray((0 for n in range(6)))
    self.IMU.mem_read(euler, self.addr, IMU.REG_EULER)
    eul_heading, eul_roll, eul_pitch = struct.unpack("<hhh", euler) 
    eul_heading /= 16
    eul_roll /= 16
    eul_pitch /= 16
    return (eul_heading, eul_pitch, eul_roll)

  def read_euler_heading(self):
    euler_x, euler_y, euler_z = self.read_euler()
    return (euler_x)
  
  def read_angular_velocity(self):
    gyro = bytearray((0 for n in range(6)))
    self.IMU.mem_read(gyro, self.addr, IMU.REG_GYRO)
    gyro_x, gyro_y, gyro_z = struct.unpack("<hhh", gyro)
    return (gyro_x, gyro_y, gyro_z)

  def read_angular_yaw_rate(self):
    roll_rate, pitch_rate, yaw_rate = self.read_angular_velocity()
    return yaw_rate
  
  def read_mag(self): 
    mag = bytearray((0 for n in range(6)))
    self.IMU.mem_read(mag, self.addr, IMU.REG_MAG)
    mag_x, mag_y, mag_z = struct.unpack("<hhh", mag)
    return (mag_x, mag_y, mag_z)
  
  def read_mag_x(self):
    mag_x, mag_y, mag_z = self.read_mag()
    return mag_x

if __name__ == "__main__":
  def oldmain():
    IMU_obj = IMU()
    IMU_obj.set_mode(IMU.NDOF)
    if IMU.file_exists('calibrations.bin'):
      IMU_obj.write_calibration_coeff()
      print("Calibration data obtained through file")
    else:
      print("Manually calibrate the IMU")
      while True: 
        yes = IMU_obj.calibration_status()
        print(yes)
        if yes == 1:
          IMU_obj.retrieve_calibration_coeff()
          print("Calibration complete and written to file")
          break
        sleep_ms(1000)

  oldmain()
  #IMU_obj = IMU()
  #IMU_obj.startup()