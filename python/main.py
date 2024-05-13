#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_msg = Twist()

    def move(self, linear_vel, angular_vel):
        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel
        self.velocity_publisher.publish(self.cmd_vel_msg)

if __name__ == '__main__':
    try:
        robot = RobotController()
        # Example: Move forward at 0.2 m/s and turn left at 0.5 rad/s
        robot.move(0.2, 0.5)
        rospy.sleep(2)  # Move for 2 seconds
        # Stop the robot
        robot.move(0, 0)
    except rospy.ROSInterruptException:
        pass
