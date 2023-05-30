#!/usr/bin/env python

import rospy
from nom_de_votre_package.srv import way_point
from std_msgs.msg import Float32

def send_set_waypoint_request(x, y):
    rospy.wait_for_service('set_waypoint_service')
    try:
        set_waypoint = rospy.ServiceProxy('set_waypoint_service', way_point)
        response = set_waypoint(x, y)
        return response.res
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))

if __name__ == '__main__':
    rospy.init_node('set_waypoint_client')

    x = 7.0  # Coordonnée x du waypoint
    y = 7.0  # Coordonnée y du waypoint

    is_moving = False  # Variable indiquant si la tortue est en mouvement (à remplacer par la valeur appropriée)

    if not is_moving:
        for _ in range(3):
            response = send_set_waypoint_request(x, y)
            if response:
                rospy.loginfo("Waypoint set successfully.")
            else:
                rospy.loginfo("Failed to set waypoint.")
    else:
        rospy.loginfo("Turtle is currently moving. Cannot set waypoint.")
