"""!
@file imu.py
@brief Provides an interface to the BNO055 IMU for orientation and calibration.
@details
  Communicates with the BNO055 sensor over I2C to read euler angles, gyroscope,
  magnetometer data, and handle calibration. Can store/read calibration data 
  from a binary file to speed up repeated usage.
"""

import pyb
from pyb import Pin, Timer, ADC, I2C
from array import array
from lib2.pin_definitions import *
from time import sleep_ms
import struct
import os
import time

class IMU:
    """!
    @class IMU
    @brief A class for interacting with a BNO055 IMU over I2C.
    @details 
      Supports reading euler angles, gyro, magnetometer, and storing/retrieving 
      calibration data in a local file named 'calibrations.bin'.
    """

    # Operating mode register and possible modes
    REG_OPR_MODE = 0x3D
    CONFIG    = 0x00
    IMU       = 0x08
    COMPASS   = 0x09
    M4G       = 0x0A
    NDOF_FMC_OFF = 0x0B
    NDOF      = 0x0C

    # Calibration register addresses
    REG_CALIB_STAT = 0x35
    REG_CAL_DATA_START = 0x55  # Start of calibration data registers

    # Euler angle register start
    REG_EULER = 0x1A

    # Gyroscope register start
    REG_GYRO = 0x14

    # Magnetometer register start
    REG_MAG = 0x0E

    def __init__(self, id=2, addr=0x28):
        """!
        @brief Initializes the IMU object with I2C and default parameters.
        @param id I2C bus ID (on STM32, commonly 1 or 2).
        @param addr I2C address of the BNO055 (default 0x28).
        """
        self.IMU = I2C(id, I2C.CONTROLLER)
        self.addr = addr
        self.accel_offset = [0, 0, 0]
        self.mag_offset = [0, 0, 0]
        self.gyr_offset = [0, 0, 0]
        self.accel_rad = 0
        self.mag_rad = 0
        self.mode = IMU.NDOF
        self.file = True

    def startup(self):
        """!
        @brief Prepares the IMU with calibration data if present, else enters calibration mode.
        @details
          - If 'calibrations.bin' exists, attempts to write calibration coefficients to IMU.
          - If that fails, instructs user to calibrate manually.
          - Once calibration is done, sets operating mode to NDOF.
        @return True if startup completed successfully, False otherwise.
        """
        status = False
        if self.file and IMU.file_exists('calibrations.bin'):
            print("Calibration file found.")
            status = self.write_calibration_coeff()

        if self.file and not status:
            self.set_mode(IMU.NDOF)

        self.file = False

        if not status:
            print("Calibration file not legit/doesn't exist. Continue calibration...")
            if self.retrieve_calibration_coeff() == -1:
                return False
            print("Calibration completed.")

        print("Startup complete.")
        return True

    @staticmethod
    def file_exists(filename):
        """!
        @brief Checks if the specified file exists.
        @param filename Name of the file to check.
        @return True if file exists, False otherwise.
        """
        try:
            with open(filename, 'rb') as f:
                print("file present")
                return True
        except OSError:
            print("file not present")
            return False

    @staticmethod
    def read_file(filename):
        """!
        @brief Reads binary data from a file and returns it.
        @param filename The file to read from.
        @return The file content in bytes or None if error.
        """
        try:
            with open(filename, 'rb') as f:
                data = f.read()
            print("File content (in bytes):", data)
            return data
        except OSError:
            print(f"Error: The file {filename} does not exist or cannot be accessed.")
            return None

    def set_mode(self, mode):
        """!
        @brief Changes the operating mode of the BNO055.
        @param mode One of the mode constants (e.g. IMU.NDOF, IMU.CONFIG, etc.).
        """
        self.mode = mode
        self.IMU.mem_write(mode, self.addr, IMU.REG_OPR_MODE)
        sleep_ms(100)

    def calibration_status(self):
        """!
        @brief Reads the calibration status register.
        @details 
          If it returns 0xFF, it might indicate an error. 
          This function also prints bit-level data for debugging.
        @return 1 if calibration register is 0xFF, else 0.
        """
        buf = bytearray(1)
        self.IMU.mem_read(buf, self.addr, IMU.REG_CALIB_STAT)
        bits = [((b >> i) & 1) for b in buf for i in range(7, -1, -1)]
        print(bits)

        compare = bytearray(1)
        compare[0] = 0xff
        if buf == compare:
            return 1
        return 0

    def retrieve_calibration_coeff(self):
        """!
        @brief Reads calibration coefficients from the IMU and saves them to 'calibrations.bin'.
        @details 
          The IMU must be fully calibrated prior to calling this function 
          (checks calibration_status first). 
          Writes data to local file and verifies by reading back.
        @return -1 if calibration incomplete, else 0 or None.
        """
        if self.calibration_status() == 0:
            print("Calibration not completed. Exiting.")
            return -1

        self.set_mode(IMU.CONFIG)
        offset = bytearray(22)
        self.IMU.mem_read(offset, self.addr, IMU.REG_CAL_DATA_START)
        try:
            accel_x, accel_y, accel_z, \
            mag_x, mag_y, mag_z, \
            gyr_x, gyr_y, gyr_z, \
            accel_rad, mag_rad = struct.unpack("<hhhhhhhhhhh", offset)

            self.accel_offset = [accel_x, accel_y, accel_z]
            self.mag_offset = [mag_x, mag_y, mag_z]
            self.gyr_offset = [gyr_x, gyr_y, gyr_z]
            self.accel_rad = accel_rad
            self.mag_rad = mag_rad

            calibration_data = (accel_x, accel_y, accel_z, mag_x, mag_y, mag_z,
                                gyr_x, gyr_y, gyr_z, accel_rad, mag_rad)

            with open('calibrations.bin', 'wb') as f:
                f.write(struct.pack("<hhhhhhhhhhh", *calibration_data))
                print("Calibration data written to binary file.")
            
            self.set_mode(self.mode)
        except struct.error as e:
            print(f"Error unpacking calibration data: {e}")
        except IOError as e:
            print(f"Error writing to binary file: {e}")

        try:
            with open('calibrations.bin', 'rb') as f:
                read_data = f.read(22)
                unpacked_data = struct.unpack("<hhhhhhhhhhh", read_data)
                print(f"Data read from binary file: {unpacked_data}")
        except IOError as e:
            print(f"Error reading from binary file: {e}")

    def write_calibration_coeff(self, filename='calibrations.bin'):
        """!
        @brief Writes calibration coefficients from a local file to the IMU.
        @details 
          Sets IMU to CONFIG mode, reads 'calibrations.bin', and writes 
          those bytes directly to the sensor's calibration registers. 
          Then restores the previous mode.
        @param filename Name of the calibration file (default 'calibrations.bin').
        @return True if successful, False otherwise.
        """
        success = False
        self.set_mode(IMU.CONFIG)
        try:
            with open(filename, 'rb') as f:
                data = f.read()
            print("File content (in bytes):", data)
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
        """!
        @brief Reads Euler angles (heading, pitch, roll) from the IMU.
        @details 
          Each angle is scaled by 1/16. heading is eul_heading, pitch is eul_pitch, roll is eul_roll.
        @return (heading, pitch, roll) in degrees.
        """
        euler = bytearray(6)
        self.IMU.mem_read(euler, self.addr, IMU.REG_EULER)
        eul_heading, eul_roll, eul_pitch = struct.unpack("<hhh", euler)
        eul_heading /= 16
        eul_roll /= 16
        eul_pitch /= 16
        return (eul_heading, eul_pitch, eul_roll)

    def read_euler_heading(self):
        """!
        @brief Convenience function to get just the heading (X Euler angle).
        @return Heading in degrees.
        """
        euler_x, euler_y, euler_z = self.read_euler()
        return euler_x

    def read_angular_velocity(self):
        """!
        @brief Reads gyroscope data (roll rate, pitch rate, yaw rate).
        @return (gyro_x, gyro_y, gyro_z) as signed 16-bit integers.
        """
        gyro = bytearray(6)
        self.IMU.mem_read(gyro, self.addr, IMU.REG_GYRO)
        gyro_x, gyro_y, gyro_z = struct.unpack("<hhh", gyro)
        return (gyro_x, gyro_y, gyro_z)

    def read_angular_yaw_rate(self):
        """!
        @brief Returns only the yaw rate from the gyro data.
        @return Yaw rate in IMU units (raw).
        """
        roll_rate, pitch_rate, yaw_rate = self.read_angular_velocity()
        return yaw_rate

    def read_mag(self):
        """!
        @brief Reads raw magnetometer data (X, Y, Z).
        @return (mag_x, mag_y, mag_z) as signed 16-bit integers.
        """
        mag = bytearray(6)
        self.IMU.mem_read(mag, self.addr, IMU.REG_MAG)
        mag_x, mag_y, mag_z = struct.unpack("<hhh", mag)
        return (mag_x, mag_y, mag_z)

    def read_mag_x(self):
        """!
        @brief Convenience function to get just the X-axis magnetometer reading.
        @return X-axis magnetometer raw value.
        """
        mag_x, mag_y, mag_z = self.read_mag()
        return mag_x


if __name__ == "__main__":
    """!
    @brief Demonstration or testing routine if imu.py is run directly.
    """
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
    # Or: IMU_obj = IMU(); IMU_obj.startup()
