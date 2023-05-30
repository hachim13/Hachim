#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from nom_de_votre_package.srv import way_point

class SetWaypointNode:
    def __init__(self):
        rospy.init_node('set_waypoint_node')

        self.pose = Pose()
        self.waypoint = (7.0, 7.0)

        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.1)
        self.is_moving = False

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.is_moving_pub = rospy.Publisher('is_moving', Bool, queue_size=1)
        rospy.Subscriber('pose', Pose, self.pose_callback)

        self.set_waypoint_service = rospy.Service('set_waypoint_service', way_point, self.set_waypoint_handler)

        self.kp = rospy.get_param('~kp', 0.5)
        self.kpl = rospy.get_param('~kpl', 0.2)

        self.run()

    def pose_callback(self, data):
        self.pose = data

    def calculate_desired_angle(self):
        dx = self.waypoint[0] - self.pose.x
        dy = self.waypoint[1] - self.pose.y
        return math.atan2(dy, dx)

    def calculate_linear_error(self):
        distance = math.sqrt((self.waypoint[0] - self.pose.x) ** 2 + (self.waypoint[1] - self.pose.y) ** 2)
        return distance

    def publish_twist(self, angular_vel):
        twist = Twist()
        twist.linear.x = self.kpl * self.calculate_linear_error()
        twist.angular.z = self.kp * angular_vel
        self.cmd_pub.publish(twist)

    def set_waypoint_handler(self, req):
        self.waypoint = (req.x.data, req.y.data)
        return True

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            desired_angle = self.calculate_desired_angle()
            angular_error = desired_angle - self.pose.theta

            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi

            if abs(angular_error) > 0.1:
                self.is_moving = True
                self.publish_twist(angular_error)
            else:
                linear_error = self.calculate_linear_error()
                if linear_error > self.distance_tolerance:
                    self.is_moving = True
                    self.publish_twist(0)
                else:
                    self.is_moving = False
                    self.publish_twist(0)

            self.is_moving_pub.publish(Bool(self.is_moving))

            rate.sleep()


if __name__ == '__main__':
    SetWaypointNode()
