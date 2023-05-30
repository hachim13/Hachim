#!/usr/bin/env python

 

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, atan, tan

 

# Variables globales
turtle_pose = Pose()
waypoint = (7, 7)

 


def pose_callback(data):
    # Mettre à jour la pose de la tortue
    global turtle_pose
    turtle_pose = data

 


def calculate_desired_angle():
    # Calculer l'angle désiré en utilisant la formule atan2
    x_diff = waypoint[0] - turtle_pose.x
    y_diff = waypoint[1] - turtle_pose.y
    desired_angle = atan2(y_diff, x_diff)
    return desired_angle

 


def calculate_command_angle(desired_angle):
    # Calculer l'angle de commande en utilisant la formule atan(tan(desired_angle - turtle_pose.theta))
    command_angle = atan(tan(desired_angle - turtle_pose.theta))
    return command_angle

 


def regulate_in_heading(desired_angle, kp):
    # Régulation en cap - Calculer la commande angulaire
    command_angle = calculate_command_angle(desired_angle)
    cmd_vel = Twist()
    cmd_vel.angular.z = kp * command_angle

 

    # Publier sur le topic cmd_vel
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub.publish(cmd_vel)

 


def set_way_point():
    # Définir le waypoint
    global waypoint
    waypoint = (7, 7)

 


def main():
    rospy.init_node('set_way_point_node', anonymous=True)

 

    # S'abonner au topic "pose"
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

 

    # Définir le waypoint
    set_way_point()

 

    # Constante de proportionnalité pour la régulation en cap
    kp = rospy.get_param('~kp', 1.0)

 

    rate = rospy.Rate(10)  # 10 Hz

 

    while not rospy.is_shutdown():
        desired_angle = calculate_desired_angle()

 

        # Régulation en cap
        regulate_in_heading(desired_angle, kp)

 

        rate.sleep()

 


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass