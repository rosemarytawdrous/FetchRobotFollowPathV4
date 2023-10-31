import rospy
from geometry_msgs.msg import Vector3Stamped, Twist
from sensor_msgs.msg import LaserScan
import math

class GuiderFollow:
    def __init__(self):
        rospy.init_node('guider_follow_node', anonymous=True)
        
        # Subscribers
        self.marker_sub = rospy.Subscriber("/aruco_single/position", Vector3Stamped, self.marker_callback)
        self.laser_sub = rospy.Subscriber("/base_scan_raw", LaserScan, self.laser_callback)
        
        # Publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Class variables
        self.twist_msg = Twist()
        self.obstacle_detected = False
        self.marker_detected = False
        self.start_time = rospy.Time.now()
        self.duration = rospy.Duration()
        
    def marker_callback(self, msg):
        distance_to_marker = round(math.sqrt(msg.vector.x ** 2 + msg.vector.y ** 2), 1)
        
        if distance_to_marker < 1.0:  # Assuming 1 meter is the safe distance to stop
            self.marker_detected = True
            self.twist_msg.linear.x = 0.0  # Stop
            self.twist_msg.angular.z = 0.0  # Stop
        else:
            self.marker_detected = False
            self.twist_msg.linear.x = 0.5  # Move forward
            self.twist_msg.angular.z = 0.0  # No rotation

        self.vel_pub.publish(self.twist_msg)

    def laser_callback(self, msg):
        for reading in msg.ranges:
            if reading < 1.0:  # Assuming 1 meter is the minimum safe distance
                self.obstacle_detected = True
                self.twist_msg.linear.x = 0.0  # Stop
                self.twist_msg.angular.z = 0.0  # Stop
                self.vel_pub.publish(self.twist_msg)
                return

        self.obstacle_detected = False

    def stop(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            self.duration = current_time - self.start_time

            if self.duration.to_sec() > 20.0:
                self.start_time = rospy.Time.now()
                self.twist_msg.linear.x = 0.0  # Stop
                self.twist_msg.angular.z = 0.0  # Stop
                self.vel_pub.publish(self.twist_msg)

if __name__ == '__main__':
    guider_follow = GuiderFollow()
    guider_follow.stop()
    rospy.spin()
