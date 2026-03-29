#!/usr/bin/env python3

"""@package docstring
Teleports the robot around with the camera facing the clue boards and collects pictures.

For each clue board, we can specify the space of relative positions and orientations
from which our text detection algorithm should be able to read the sign.

`collect_images.py` will teleport the floating eyeinthesky_ghost model around within the
specfied ranges and collect images from the robot camera. To avoid training bias, the
camera will collect roughly equal numbers of images from each clue board.
"""

import rospy
import numpy as np
import tf.transformations as tr
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Point

rospy.init_node('collect_images')

# TODO