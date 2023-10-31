#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GuiderJoyTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('guider_joy_teleop', anonymous=True)
        
        # Subscriber for joystick messages
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Twist message to store velocity commands
        self.twist_msg = Twist()

    def joy_callback(self, msg):
        # Callback function for joystick messages
        # Linear velocity is controlled by the second axis of the joystick
        self.twist_msg.linear.x = msg.axes[1] * 2  # Multiply by 2 for scaling
        
        # Angular velocity is controlled by the fourth axis of the joystick
        self.twist_msg.angular.z = msg.axes[3] * 2  # Multiply by 2 for scaling

    def velocity_publisher(self):
        # Function to publish velocity commands
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Create an instance of the class
        teleop = GuiderJoyTeleop()
        
        # Start the velocity publisher
        teleop.velocity_publisher()
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
