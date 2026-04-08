#!/usr/bin/env python3

"""@package docstring
Teleports the robot around with the camera facing the clue boards and collects pictures.

For each clue board, we can specify the space of relative positions and orientations
from which our text detection algorithm should be able to read the sign.

`collect_images.py` will teleport the floating eyeinthesky_ghost model around within the
specfied ranges and collect images from the robot camera. To avoid training bias, the
camera will collect roughly equal numbers of images from each clue board.

Date: 2026-03-29
Author: Jonah Lee
"""

import tf.transformations as tr
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from move_relative import (
    move_model_relative,
    DEFAULT_WORLD_POSE,
    DEFAULT_ORIENTATION,
    DEFAULT_XYZ,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from datetime import datetime
from pathlib import Path
import numpy as np
import logging
import random
import rospy
import time
import sys
import os
import csv


def set_up_logging(logger_name, log_dir: Path, log_level=logging.DEBUG) -> logging.Logger:
    """
    [Written by Gemini, modified by Jonah Lee]

    Sets up a logger with a FileHandler and a StreamHandler.
    """
    # 1. Create directory if it doesn't exist
    if not os.path.exists(str(log_dir)):
        os.makedirs(str(log_dir))

    # 2. Create the logger
    logger = logging.getLogger(logger_name)
    # FIX: since other nodes use logging as well, we don't want to inherit
    #      the properties of the root logger so we set propagate to false
    logger.propagate = False
    logger.setLevel(log_level)

    # Avoid duplicate handlers if the function is called twice
    if logger.hasHandlers():
        return logger

    # 3. Create formatters
    # Detailed for files, concise for the terminal
    file_fmt = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')
    console_fmt = logging.Formatter('%(levelname)s: %(message)s')

    log_file = os.path.join(log_dir, f"{logger_name}.log")
    
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(log_level)
    file_handler.setFormatter(file_fmt)

    # 5. Console Handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO) # Keep terminal clean with INFO+
    console_handler.setFormatter(console_fmt)

    # 6. Add handlers
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger

def save_camera_view(bridge: CvBridge, save_path: Path, topic: str, move_time) -> None:
    """
    Saves a the latest picture from a ros topic to a file

    Raises an error if `rospy.wait_for_message` fails.
    """
    while True:
        msg = rospy.wait_for_message(topic, Image)
        # Check if the image timestamp is newer than the teleport command
        if msg.header.stamp > move_time:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(str(save_path), cv_image)
            break


def get_orientation_to_face(target_pos: Point, seeker_pos: Point) -> Quaternion:
    """
    Calculates orientation so seeker faces target in 3D.
    target_pos: geometry_msgs/Point (model_1)
    seeker_pos: geometry_msgs/Point (model_2)
    """
    dx = target_pos.x - seeker_pos.x
    dy = target_pos.y - seeker_pos.y
    dz = target_pos.z - seeker_pos.z

    # Distance in the XY plane (ground distance)
    ground_dist = np.sqrt(dx**2 + dy**2)

    # 1. Yaw: Left/Right rotation
    yaw = np.arctan2(dy, dx)

    # 2. Pitch: Up/Down rotation
    # Note: We use -dz because a positive pitch in ROS usually 
    # points the X-axis "down" toward the ground.
    pitch = np.arctan2(-dz, ground_dist)

    # 3. Roll: Usually 0 for a "Look-At" behavior
    roll = 0

    # Convert to ROS Quaternion [x, y, z, w]
    q_array = quaternion_from_euler(roll, pitch, yaw)

    # Construct the message
    return Quaternion(x=q_array[0], y=q_array[1], z=q_array[2], w=q_array[3])


if __name__ == '__main__':

    # ==================================
    #         DEFINE CONSTANTS
    # ==================================

    NUM_IMAGES = 100
    CLUE_BOARD_MODEL_NAMES = [f"car{x}" for x in range(8)] # car0 - car7
    ROBOT_MODEL_NAME = 'B1'
    CAMERA_TOPIC = "/B1/rrbot/camera1/image_raw"

    DEG_TO_RAD = np.pi / 180

    ASPECT_RATIO = 16.0 / 9.0
    FOV_HORIZ_DEG = 120
    FOV_VERT_DEG = 120 / ASPECT_RATIO

    # The clue board center must be at most this many degrees
    # from the edge of the camera view
    CAMERA_FOV_PAD_DEG = 30
    # (-H_SPAN, H_SPAN) and (-V_SPAN, V_SPAN) define a bounding box
    # for horizontal, vertical angles such that the object is within
    # CAMERA_FOV_PAD_DEG degrees of the edge of the frame
    H_SPAN = FOV_HORIZ_DEG / 2 - CAMERA_FOV_PAD_DEG
    V_SPAN = FOV_VERT_DEG / 2 - CAMERA_FOV_PAD_DEG

    # Robot position wrt. board
    RADIUS_RANGE = (0.4, 1.2)
    
    THETA_RANGE_DEG = (-30, 30)  # Azimuthal angle
    PHI_RANGE_DEG = (65, 90)  # Polar angle, enforce <= 90
    THETA_OFFSET = 90 * DEG_TO_RAD # IDK why this is needed

    # Robot orientation wrt facing board.
    ROLL_RANGE_DEG = (-5, 5)
    PITCH_RANGE_DEG = (-H_SPAN, H_SPAN)
    YAW_RANGE_DEG = (V_SPAN, V_SPAN)

    # Clueboard information
    PATH_TO_CLUES_CSV = '/home/fizzer/ros_ws/src/2025_competition/enph353/enph353_gazebo/scripts/plates.csv'
    clue_list = []
    with open(PATH_TO_CLUES_CSV, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Check if the row is not empty
            if row:
                clue_list.append(row[1]) # Column 2
    

    # ==================================
    #           SET UP OUTPUT
    # ==================================
    
    rospy.init_node('collect_images', anonymous=True)

    repo_root: Path = Path(__file__).parent.parent.parent.parent
    data_dir = repo_root / 'data'

    timestamp = datetime.now().strftime(r"%d_%m_%Y_%H_%M_%S")
    out_dir_name = "images_raw_" + timestamp
    out_dir = data_dir / out_dir_name

    # Create directory
    os.makedirs(out_dir, exist_ok=True)

    # Start logging to file & console
    logger = set_up_logging('collect_images', out_dir)

    # Log all config constants (may generate some garbage)
    constants: dict = {k:v for k, v in locals().items() if k.isupper()}
    for k, v in constants.items():
        logger.info(f"{k}={v}")

    # Instantiate CvBridge for converting to cv images.
    bridge = CvBridge()

    # ==================================
    #       IMAGE COLLECTION LOOP
    # ==================================

    for i in range(NUM_IMAGES):

        # Choose a random clue board
        board_num = random.randint(0,7)
        #ref_model_name = random.choice(CLUE_BOARD_MODEL_NAMES)
        ref_model_name = f"car{board_num}"
        logger.info(f"Selected new reference model: '{ref_model_name}'")

        # Select a random theta, phi, r position relative to the clue board
        theta = random.uniform(*THETA_RANGE_DEG) * DEG_TO_RAD + THETA_OFFSET
        phi = random.uniform(*PHI_RANGE_DEG) * DEG_TO_RAD
        r = random.uniform(*RADIUS_RANGE)
        # Select a random roll, pitch, yaw orientation relative to facing the clue board
        roll_rel = random.uniform(*ROLL_RANGE_DEG) * DEG_TO_RAD
        pitch_rel = random.uniform(*PITCH_RANGE_DEG) * DEG_TO_RAD
        yaw_rel = random.uniform(*YAW_RANGE_DEG) * DEG_TO_RAD

        logger.info("Selected new relative position / orientation!")
        logger.info(f"theta={theta}")
        logger.info(f"phi={phi}")
        logger.info(f"r={r}")
        logger.info(f"roll_rel={roll_rel}")
        logger.info(f"pitch_rel={pitch_rel}")
        logger.info(f"yaw_rel={yaw_rel}")

        # Compute relative position
        x = r*np.sin(phi)*np.cos(theta)
        y = r*np.sin(phi)*np.sin(theta)
        z = r*np.cos(phi)
        rel_position = Point(x, y, z)

        # Compute relative orientation
        clue_board_pos = Point(0, 0, 0)  # Since we are working in the clue board frame
        orientation_to_face: Quaternion = get_orientation_to_face(clue_board_pos, rel_position)
        rel_orientation = Quaternion(
            *tr.quaternion_from_euler(
                roll_rel,
                pitch_rel,
                yaw_rel,
                'rxyz'
            )
        )
        q_total = tr.quaternion_multiply(
            [orientation_to_face.x, orientation_to_face.y, orientation_to_face.z, orientation_to_face.w],
            [rel_orientation.x, rel_orientation.y, rel_orientation.z, rel_orientation.w]
        )
        orientation = Quaternion(*q_total)

        # Combine position + orientation
        rel_pose = Pose(rel_position, orientation)
        logger.info(f"rel_pose={rel_pose}")

        # Teleport to the new pose
        move_model_relative(ROBOT_MODEL_NAME, rel_pose, ref_model_name)

        # Get the current ROS time AFTER the move
        move_time = rospy.get_rostime()

        # Save a picture
        current_clue = clue_list[board_num]
        file_name = f"image_{i}_{board_num}_" + current_clue + ".png"
        file_path = out_dir / file_name
        try:
            save_camera_view(bridge, file_path, CAMERA_TOPIC, move_time)
            logger.info(f"Saved camera view to {out_dir_name}/{file_name}!")
        except Exception as e:  
            logger.warning("save_camera_view failed!")
            logger.warning(e)

        # Comment this out to skip sleep 
        # time.sleep(1)

    logger.info(f"Image collection complete! Collected {NUM_IMAGES} images.")
    logger.info(f"Results are in: {out_dir}")