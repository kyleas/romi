"""!
@file romi.py
@brief Provides high-level kinematic calculations for the ROMI robot.
@details
  The Romi class calculates the robot's linear and angular motion (x, y, heading) 
  using left/right wheel velocities. It contains methods to calculate velocity, 
  reset distances, and handle heading offsets.
"""

import math
from time import ticks_ms, ticks_diff

class Romi:
    """!
    @class Romi
    @brief Encapsulates the ROMI chassis kinematics and state.
    @details
      Stores parameters like wheel radius and axle width. Keeps track of position,
      heading, and distance traveled, updating them based on current wheel velocities.
    """

    def __init__(self):
        """!
        @brief Constructs the Romi object with default geometry and initial states.
        """
        self.width = 0.141       # Distance between wheels (m)
        self.r_w = 0.036        # Wheel radius (m)
        self.left_velocity = 0
        self.right_velocity = 0
        self.left_position = 0
        self.right_position = 0
        self.prev_time = 0
        self.angle = 0
        self.linear_distance = 0
        self.xpos = 0
        self.ypos = 0

    def zero_heading(self, heading):
        """!
        @brief Sets an offset for the IMU heading so the robot can be oriented to 0.
        @param heading The current heading reading from the IMU, used as the new 0 reference.
        """
        self.heading_offset = heading
        print(f"Heading offset: {self.heading_offset}")

    def update_time(self):
        """!
        @brief Records the current time for subsequent motion calculations.
        """
        self.prev_time = ticks_ms()

    def update(self, left_velocity, right_velocity, heading):
        """!
        @brief Updates the robot's estimated position and heading.
        @details
          Calculates linear velocity and angular velocity from the left/right velocities.
          Then integrates over the elapsed time to update x/y coordinates, heading, 
          and total linear distance.
        
        @param left_velocity  Left wheel velocity in [RPM] (or similar).
        @param right_velocity Right wheel velocity in [RPM].
        @param heading        The current absolute heading (deg), typically from an IMU.
        """
        self.left_velocity = left_velocity
        self.right_velocity = right_velocity

        linear_velocity = self.calc_linear_velocity(left_velocity, right_velocity)
        angular_velocity = self.calc_angular_velocity(left_velocity, right_velocity)

        current_time = ticks_ms()
        dt = ticks_diff(current_time, self.prev_time) / 1000
        self.prev_time = current_time

        self.angle = heading - self.heading_offset

        x_vel = linear_velocity * math.cos(self.angle)
        y_vel = linear_velocity * math.sin(self.angle)

        self.xpos += x_vel * dt
        self.ypos += y_vel * dt

        prev_lin_distance = self.linear_distance
        self.linear_distance += (linear_velocity * dt)

        # If there's an unreasonable jump, revert it (example logic)
        if abs(self.linear_distance - prev_lin_distance) > 1:
            print("Reset lin distance")
            self.linear_distance = prev_lin_distance

    def update_angle(self, heading):
        """!
        @brief Updates only the angle, skipping linear distance computations.
        @param heading The current absolute heading from the IMU.
        """
        self.angle = heading - self.heading_offset

    def calc_des_wheel_vel(self, linear, angular, MAX_VEL):
        """!
        @brief Converts desired chassis linear/angular speeds into left/right wheel RPMs.
        @details
          Also checks against a max velocity limit (if you decide to implement it).
        
        @param linear  Desired linear velocity in m/s.
        @param angular Desired angular velocity in rad/s.
        @param MAX_VEL A shared variable holding the maximum allowed wheel RPM.
        @return (left_rpm, right_rpm) as a tuple of floats in RPM.
        """
        max_vel = MAX_VEL.get()

        # Convert linear and angular velocities to wheel RPM
        left = (linear / self.r_w - angular * self.width / 2 / self.r_w) * 9.5492968
        right = (linear / self.r_w + angular * self.width / 2 / self.r_w) * 9.5492968

        print(f"linear: {linear/self.r_w}, angular: {angular * self.width / 2 / self.r_w}")
        # Example limit check (commented out by default)
        # left = max(min(left, max_vel), -max_vel)
        # right = max(min(right, max_vel), -max_vel)

        return (left, right)

    def calc_linear_velocity(self, left, right):
        """!
        @brief Computes the robot's linear velocity from wheel RPMs.
        @param left  Left wheel velocity in RPM.
        @param right Right wheel velocity in RPM.
        @return Robot linear velocity in m/s.
        """
        return (left + right) * self.r_w / 2 / 9.5492968

    def calc_angular_velocity(self, left, right):
        """!
        @brief Computes the robot's angular velocity from wheel RPMs.
        @param left  Left wheel velocity in RPM.
        @param right Right wheel velocity in RPM.
        @return Robot angular velocity in rad/s.
        """
        return (right - left) / 9.5492968 / self.width * self.r_w

    def get_distance(self):
        """!
        @brief Returns the integrated linear distance traveled.
        @return The total linear distance in meters since last reset.
        """
        return self.linear_distance

    def zero_distance(self):
        """!
        @brief Resets the integrated linear distance to 0.
        @details 
          Also updates prev_time to current time, ensuring subsequent distance calculations
          start from this moment.
        """
        self.linear_distance = 0
        self.prev_time = ticks_ms()

    def get_angle(self):
        """!
        @brief Returns the current heading angle offset by heading_offset.
        @return Current heading in degrees (approx).
        """
        return self.angle

    def set_distance(self, distance):
        """!
        @brief Forces the integrated linear distance to a specific value.
        @param distance The new distance to set (meters).
        """
        self.linear_distance = distance
