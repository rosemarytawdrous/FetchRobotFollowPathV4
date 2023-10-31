import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class EnvironmentSensing:
    def __init__(self):
        rospy.init_node('environment_sensing_node', anonymous=True)
        
        # Subscriber
        self.sub1 = rospy.Subscriber("base_scan_raw", LaserScan, self.laser_callback)
        
        # Class variables
        self.laser_readings = 0.0
        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Method created to detect obstacles and get laser readings
        self.obstacle_detected = self.detect_obstacle(msg)
        self.laser_readings = self.get_laser_readings(msg)

        if self.obstacle_detected:
            rospy.loginfo("Obstacle detected. Stopping.")
        else:
            rospy.loginfo("No obstacle. Proceed.")

    def detect_obstacle(self, msg):
        # Logic to detect obstacles based on laser scan
        for reading in msg.ranges:
            if reading < 1.0:  # Assuming 1 meter is the minimum safe distance
                return True
        return False

    def get_laser_readings(self, msg):
        # Logic to get laser readings
        min_reading = min(msg.ranges)
        return min_reading

    def status(self):
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected. Stopping.")
            else:
                rospy.loginfo("No obstacle. Proceed.")

if __name__ == '__main__':
    environment_sensing = EnvironmentSensing()
    environment_sensing.status()
    rospy.spin()
