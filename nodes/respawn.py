#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_ros.gazebo_interface import SetModelState
from math import sqrt


def spawn_position(position=None):

        msg = ModelState()
        msg.model_name = 'B1'

        if position is None:
            # Default: -x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57
            # Note: position is a quaternion. I have converted the default position.
            position = [5.5, 2.5, 0.2, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2]

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

if __name__ == "__main__":
     spawn_position()