class LineFollow:

    def __init__(self, IR_SENSOR):
        self.IR_SENSOR = IR_SENSOR

    def update(self, MAX_VEL):
        # Get centroid position (-1 to 1)
        centroid = self.IR_SENSOR.calc_centroid()
            
        # Get max velocity from share
        max_vel = MAX_VEL.get()
        
        # Calculate motor velocities based on centroid
        # When centroid is 0, both motors run at max_vel (straight)
        # When centroid is -1, right motor runs at max_vel, left at -max_vel (sharp left turn)
        # When centroid is 1, left motor runs at max_vel, right at -max_vel (sharp right turn)
        # print(f"Centroid: {centroid}")
        left_vel = max_vel * (1 + centroid) 
        right_vel = max_vel * (1 - centroid)

        if left_vel > max_vel:
            left_vel = max_vel
        if right_vel > max_vel:
            right_vel = max_vel

        if left_vel < 0:
            left_vel = 0
        if right_vel < 0:
            right_vel = 0

        return left_vel, right_vel