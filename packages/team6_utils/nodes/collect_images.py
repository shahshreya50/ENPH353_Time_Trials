#!/usr/bin/env python3

"""@package docstring
Teleports the robot around with the camera facing the clue boards and collects pictures.

For each clue board, we can specify the space of relative positions and orientations
from which our text detection algorithm should be able to read the sign.

`collect_images.py` will teleport the floating eyeinthesky_ghost model around within the
specfied ranges and collect images from the robot camera. To avoid training bias, the
camera will collect roughly equal numbers of images from each clue board.
"""

import tf.transformations as tr
from geometry_msgs.msg import Pose, Quaternion, Point
from pathlib import Path
from move_relative import (
    move_model_relative,
    DEFAULT_WORLD_POSE,
    DEFAULT_ORIENTATION,
    DEFAULT_XYZ,
)
import numpy as np
import random
import rospy
import time
import os


if __name__ == '__main__':

    # ==================================
    #         DEFINE CONSTANTS
    # ==================================

    NUM_IMAGES = 100
    CLUE_BOARD_MODEL_NAMES = [f"car{x}" for x in range(8)] # car0 - car7
    ROBOT_MODEL_NAME = 'B1'

    DEG_TO_RAD = np.pi / 180

    CAMERA_FOV_DEG = 120

    # Robot position wrt. board
    R_RANGE_M = (0.4, 0.4)
    THETA_RANGE_DEG = (0, 0)  # Azimuthal angle
    PHI_RANGE_DEG = (90, 90)  # Polar angle
    THETA_OFFSET = 90 * DEG_TO_RAD

    # Robot orientation wrt facing board.
    PITCH_RANGE_DEG = (0, 0)
    ROLL_RANGE_DEG = (0, 0)
    YAW_RANGE_DEG = (0, 0)
    YAW_OFFSET = -90 * DEG_TO_RAD

    # ==================================
    #         SET UP OUTPUT DIR
    # ==================================
    
    rospy.init_node('collect_images')

    repo_root: Path = Path(__file__).parent.parent.parent.parent
    data_dir = repo_root / 'data'

    os.makedirs(data_dir, exist_ok=True)

    # ==================================
    #       IMAGE COLLECTION LOOP
    # ==================================

    for i in range(NUM_IMAGES):

        # Choose a random clue board
        ref_model_name = random.choice(CLUE_BOARD_MODEL_NAMES)

        # Select a random theta, phi, r position relative to the clue board
        theta = random.uniform(*THETA_RANGE_DEG) * DEG_TO_RAD + THETA_OFFSET
        phi = random.uniform(*PHI_RANGE_DEG) * DEG_TO_RAD
        r = random.uniform(*R_RANGE_M)

        # Select a random roll, pitch, yaw orientation relative to the clue board
        roll = random.uniform(*ROLL_RANGE_DEG) * DEG_TO_RAD
        pitch = random.uniform(*PITCH_RANGE_DEG) * DEG_TO_RAD
        yaw = random.uniform(*YAW_RANGE_DEG) * DEG_TO_RAD

        # Construct a rlative Pose object
        x = r*np.sin(phi)*np.cos(theta)
        y = r*np.sin(phi)*np.sin(theta)
        z = r*np.cos(phi)
        rel_position = Point(x, y, z)
        rel_orientation = Quaternion(
            *tr.quaternion_from_euler(
                roll,
                pitch,
                yaw + YAW_OFFSET,
                'rxyz'
            )
        )
        rel_pose = Pose(rel_position, rel_orientation)

        # Teleport to the new pose
        move_model_relative(ROBOT_MODEL_NAME, rel_pose, ref_model_name)

        # Save a picture
        # TODO
        time.sleep(0.5)
