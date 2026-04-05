#!/usr/bin/env python3

"""
Helper script to determine the state (including pose) of a bunch of models.
I'm using this to help route paths to the different clue boards.

Date: 2026-04-04
Author: Jonah Lee
"""

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import math
from pprint import pprint




if __name__ == "__main__":

    CLUE_BOARD_MODEL_NAMES = [f"car{x}" for x in range(8)] # car0 - car7

    names_to_query = CLUE_BOARD_MODEL_NAMES + ['B1']

    position_map = {}

    rospy.init_node("get_car_positions")

    for model_name in names_to_query:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state: ModelState = get_state(model_name, 'world')

        x = state.pose.position.x
        y = state.pose.position.y
        z = state.pose.position.z
        qz = state.pose.orientation.z
        theta = math.asin(qz) * 2

        print(f"Model: {model_name}")
        print(f"Position (x, y, z, theta): {(x, y, z, theta)}")

        position_map[model_name] = (x, y, z, theta)
    
    pprint(position_map)
