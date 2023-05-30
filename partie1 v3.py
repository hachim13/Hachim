#!/usr/bin/env python

import rospy
from math import atan2, atan, tan
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class SetWaypointNode:
    def __init__(self):
        rospy.init_node('set_way_point')

        # Subscribing to the "pose" topic
        rospy.Subscriber('pose', Pose, self.pose_callback)

        # Defining the waypoint
        self.waypoint = (7, 7)

        # Getting the Kp value from the ROS parameter server
        self.Kp = rospy.get_param('~Kp', 1.0)

        # Publishing the cmd_vel topic
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Initializing the pose variable
        self.pose = Pose()

    def pose_callback(self, data):
        # Updating the pose variable
        self.pose = data

        # Calculating the desired angle
        x_diff = self.waypoint[0] - self.pose.x
        y_diff = self.waypoint[1] - self.pose.y
        desired_angle = atan2(y_diff, x_diff)

        # Calculating the error
        error = atan(tan(desired_angle - self.pose.theta))

        # Calculating the control input
        angular_velocity = self.Kp * error

        # Creating the Twist message
        cmd_msg = Twist()
        cmd_msg.angular.z = angular_velocity

        # Publishing the Twist message
        self.cmd_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        node = SetWaypointNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
