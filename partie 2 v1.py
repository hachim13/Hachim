#!/usr/bin/env python

import rospy
from math import sqrt
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SetWaypointNode:
    def __init__(self):
        rospy.init_node('set_way_point')

        # Subscribing to the "pose" topic
        rospy.Subscriber('pose', Pose, self.pose_callback)

        # Defining the waypoint
        self.waypoint = (7, 7)

        # Getting the Kp and Kpl values from the ROS parameter server
        self.Kp = rospy.get_param('~Kp', 1.0)
        self.Kpl = rospy.get_param('~Kpl', 1.0)

        # Publishing the cmd_vel topic
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Publishing the is_moving topic
        self.is_moving_pub = rospy.Publisher('is_moving', Bool, queue_size=10)

        # Initializing the pose variable
        self.pose = Pose()

        # Setting the distance tolerance
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.1)

    def pose_callback(self, data):
        # Updating the pose variable
        self.pose = data

        # Calculating the distance to the waypoint
        x_diff = self.waypoint[0] - self.pose.x
        y_diff = self.waypoint[1] - self.pose.y
        distance = sqrt((y_diff)**2 + (x_diff)**2)

        # Calculating the linear error
        linear_error = distance

        if linear_error > self.distance_tolerance:
            # Calculating the linear control input
            linear_velocity = self.Kpl * linear_error

            # Creating the Twist message
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_velocity

            # Publishing the Twist message
            self.cmd_pub.publish(cmd_msg)

            # Publishing True on the is_moving topic
            self.is_moving_pub.publish(True)
        else:
            # Publishing False on the is_moving topic
            self.is_moving_pub.publish(False)

if __name__ == '__main__':
    try:
        node = SetWaypointNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
