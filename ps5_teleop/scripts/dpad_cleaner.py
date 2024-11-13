#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class DpadCleaner:
    def __init__(self):
        rospy.init_node('dpad_cleaner')
        
        # publisher for cleaned cmd_vel
        self.clean_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Subscribe to joy topic
        rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Parameters
        self.linear_axis = 7  # Up/Down arrows
        self.angular_axis = 6  # Left/Right arrows
        self.enable_button = 4  # L1
        self.turbo_button = 6  # L2
        
        # Speeds
        self.linear_speed = 0.3
        self.angular_speed = 0.6
        self.turbo_multiplier = 2.0
        
        # Threshold for d-pad input
        self.threshold = 0.5

    def joy_callback(self, joy_msg):
        if len(joy_msg.buttons) <= self.enable_button or len(joy_msg.axes) <= max(self.linear_axis, self.angular_axis):
            return

        # Only process if enable button (L1) is pressed
        if not joy_msg.buttons[self.enable_button]:
            # Not processing any command when enable button is released
            zero_twist = Twist()
            self.clean_pub.publish(zero_twist)
            return

        # Create Twist message
        twist = Twist()
        
        # Get speed multiplier
        speed_multiplier = self.turbo_multiplier if (len(joy_msg.buttons) > self.turbo_button and 
                                                    joy_msg.buttons[self.turbo_button]) else 1.0

        # Process linear motion (up/down)
        linear_input = joy_msg.axes[self.linear_axis]
        if abs(linear_input) > self.threshold:
            twist.linear.x = self.linear_speed * linear_input * speed_multiplier
        
        # Process angular motion (left/right)
        angular_input = joy_msg.axes[self.angular_axis]
        if abs(angular_input) > self.threshold:
            twist.angular.z = self.angular_speed * angular_input * speed_multiplier

        # Publish the cleaned command
        self.clean_pub.publish(twist)

if __name__ == '__main__':
    try:
        cleaner = DpadCleaner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
