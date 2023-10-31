from sensor_msgs.msg import LaserScan

class LaserDetection:
    def __init__(self):
        self.LASER_FIELD_OF_VIEW = 140
        self.LASER_LIMIT = 0.35
        self.RAD_ROBOT = 0.257
        self.laser_reading = float('inf')  # Initialize to a large value

    def detect_obstacle(self, laser_scan: LaserScan):
        laser_reading = laser_scan.range_max  # Get max reading
        range_start = (len(laser_scan.ranges) // 2) - (self.LASER_FIELD_OF_VIEW // 2)  # Consider start of FOV
        range_end = (len(laser_scan.ranges) // 2) + (self.LASER_FIELD_OF_VIEW // 2)  # Consider end of FOV

        for i in range(range_start, range_end + 1):
            if laser_scan.ranges[i] < laser_reading:
                laser_reading = laser_scan.ranges[i]  # Store laser reading

        return laser_reading <= self.LASER_LIMIT + self.RAD_ROBOT  # Check if condition is valid

    def get_laser_reading(self, laser_scan: LaserScan):
        range_start = (len(laser_scan.ranges) // 2) - (self.LASER_FIELD_OF_VIEW // 2)
        range_end = (len(laser_scan.ranges) // 2) + (self.LASER_FIELD_OF_VIEW // 2)

        for i in range(range_start, range_end + 1):
            if laser_scan.ranges[i] < self.laser_reading:
                self.laser_reading = laser_scan.ranges[i]

        return self.laser_reading
