"""!
@file line_follow.py
@brief Implements a line-following strategy using IR sensors.
@details
  Contains a class that takes IR sensor data and outputs left/right wheel velocities
  to follow a line. The user can set a maximum velocity, and this module will 
  scale the left/right speeds based on the sensor centroid.
"""

class LineFollow:
    """!
    @class LineFollow
    @brief Class that computes left/right motor velocities to keep a line sensor's centroid at center.
    @details
      The centroid from the IR sensor is used to adjust wheel speeds in real-time,
      steering the robot to follow a line on the floor.
    """

    def __init__(self, IR_SENSOR):
        """!
        @brief Constructs the line follower object with a reference to an IR sensor module.
        @param IR_SENSOR An instance of the IR class for reading line position data.
        """
        self.IR_SENSOR = IR_SENSOR

    def update(self, MAX_VEL):
        """!
        @brief Updates and returns new left/right velocities based on IR sensor centroid.
        @details
          - Reads the centroid (-1 to +1).
          - Scales each wheel's velocity from 0 up to max_vel, with opposing directions for large errors 
            (in a more advanced scenario).
          - Clamps outputs to [0..max_vel].
        
        @param MAX_VEL A shared variable containing the maximum allowed velocity.
        @return (left_vel, right_vel) as floats in the range [0..max_vel].
        """
        centroid = self.IR_SENSOR.calc_centroid()
        max_vel = MAX_VEL.get()

        # Compute naive differential drive velocities
        left_vel = max_vel * (1 + centroid)
        right_vel = max_vel * (1 - centroid)

        # Clip or rectify to 0..max_vel
        if left_vel > max_vel:
            left_vel = max_vel
        if right_vel > max_vel:
            right_vel = max_vel
        if left_vel < 0:
            left_vel = 0
        if right_vel < 0:
            right_vel = 0

        return left_vel, right_vel
