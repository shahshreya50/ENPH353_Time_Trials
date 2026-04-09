#!/usr/bin/env python3
import rospy
import numpy as np
import math
import os
from controller import ForceController
from move_relative import respawn_model
from cv_bridge import CvBridge
from pathlib import Path
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from sign_positioning import VIEW_POSITIONS


# e.g. 'car0' -> (x, y, z, phi)
MODEL_POSITIONS: dict = {
    'B1_spawnpos': np.array([5.5, 2.5, 0.2, -1.5707963267948968]),
    'B1_grounded': np.array([5.500004840037357, 2.4999946488568625, 0.06429105580640433, -1.5700318536518063]),
    'tunnel': np.array([-3.2, -2.3, 0.3, 0]),
    'car0': np.array([5.81, 1.64, 0.04, 0.0]),
    'car1': np.array([5.16, -1.35, 0.04, 0.0]),
    'car2': np.array([4.0, -1.67, 0.04, -1.57]),
    'car3': np.array([0.83, -0.54, 0.04, 3.1399999999999753]),
    'car4': np.array([0.83, 1.5, 0.04, 0.0]),
    'car5': np.array([-3.41, 1.71, 0.04, -1.57]),
    'car6': np.array([-3.8, -2.01, 0.04, 1.57]),
    'car7': np.array([-0.9, -1.2, 1.86, 1.57]),
}

def fly_to_carx(x: int,
                ctrl: ForceController,
                drone_pos: np.ndarray = None,
                vertical_clearance: float = 1.0):

    if drone_pos is None:
        # Default: spawn point
        drone_pos: np.ndarray = MODEL_POSITIONS['B1_spawnpos']

    car_name = f"car{x}"

    # Fix the car theta so that 0 deg means the text faces +x
    end_pos: np.ndarray = VIEW_POSITIONS[car_name]

    delta_pos = end_pos - drone_pos
    delta_z = delta_pos[2]
    delta_xy = delta_pos[:2]
    delta_phi = delta_pos[3]

    ctrl.increase_position((0, 0, delta_z + vertical_clearance))
    ctrl.increase_position((*delta_xy, 0))
    ctrl.increase_angle((0, 0, delta_phi))
    ctrl.increase_position((0, 0, -vertical_clearance))

def fly_to_tunnel(ctrl: ForceController,
                  drone_pos: np.ndarray = None,
                  vertical_clearance: float = 1.0,
                  end_height_above: float = 0.1):

    if drone_pos is None:
        # Default: spawn point
        drone_pos: np.ndarray = MODEL_POSITIONS['B1_grounded']            

    tunnel_pos: np.ndarray = MODEL_POSITIONS['tunnel']

    delta_pos = tunnel_pos - drone_pos
    delta_xy = delta_pos[:2]
    delta_z = delta_pos[2]

    ctrl.increase_position((0, 0, delta_z + vertical_clearance))
    ctrl.increase_position((*(delta_xy), 0))
    ctrl.increase_position((0, 0, -vertical_clearance + end_height_above))

def wait_for_fresh_message(topic, topic_type, timeout=None):
    """
    Helper to ensure we get a message published AFTER this call.
    """
    start_time = rospy.Time.now()
    container = {'msg': None}

    def callback(msg):
        # Image messages have headers; we check if the frame is actually new
        if msg.header.stamp > start_time:
            container['msg'] = msg

    sub = rospy.Subscriber(topic, topic_type, callback)
    
    deadline = None
    if timeout:
        duration = timeout if isinstance(timeout, rospy.Duration) else rospy.Duration(timeout)
        deadline = start_time + duration

    try:
        while not rospy.is_shutdown() and container['msg'] is None:
            if deadline and rospy.Time.now() > deadline:
                raise rospy.ROSException(f"Timeout exceeded waiting for fresh image on {topic}")
            rospy.sleep(0.01)
        return container['msg']
    finally:
        sub.unregister()

def save_camera_view(bridge: CvBridge, save_path: Path, topic: str) -> Image:
    """
    Saves a fresh picture from a ros topic to a file.
    Blocks until a new message arrives.
    """
    msg = wait_for_fresh_message(topic, Image, timeout=5.0)
    
    if msg is not None:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(str(save_path), cv_image)
        return msg
    else:
        rospy.logerr("Failed to capture fresh image.")


if __name__ == "__main__":

    rospy.init_node('follow_route')

    #start publishers
    pub_clueboards = rospy.Publisher('/clueboard_images', Image, queue_size=8)
    pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)

    MODEL_MASS = 20.00
    MODEL_I_ZZ = 0.1
    A_GRAVITY = 9.8
    GOTO_TUNNEL = True
    
    ctrl = ForceController(
        MODEL_MASS,
        MODEL_I_ZZ,
        force_offsets=(0, 0, MODEL_MASS * A_GRAVITY),
        default_impulse_duration=0.02,
    )

    bridge = CvBridge()

    root: Path = Path(__file__).parent.parent.parent.parent
    data_dir: Path = root / 'data'

    timestamp = datetime.now().strftime(r"%d_%m_%Y_%H_%M_%S")
    out_dir_name = "run_outputs_" + timestamp
    out_dir: Path = data_dir / out_dir_name

    os.makedirs(out_dir, exist_ok=True)

    # ==========================================================
    # Simulate a gazebo reset
    # ==========================================================

    respawn_model('B1')
    ctrl.zero_force(with_offset=False)
    rospy.sleep(2)

    # ==========================================================
    # This is the point from which we assume the node will start
    # ==========================================================

    pub_score.publish('Team6,abcde,0,START')

    # Cancel gravity
    ctrl.zero_force(with_offset=True)

    if GOTO_TUNNEL:
        fly_to_tunnel(ctrl, end_height_above=1)
        ctrl.zero_force(with_offset=False) # Fall onto the tunnel
        rospy.sleep(1)
        ctrl.zero_force(with_offset=True)


    ordered_signs = [7, 0, 1, 2, 3, 4, 5, 6]
    for car_i in ordered_signs:        

        # Add extra clearance for hill
        vertical_clearance = 4 if car_i == 6 else 1

        respawn_model('B1')

        rospy.sleep(1) # IMPORTANT - Wait for respawn_model to finish!

        fly_to_carx(car_i, ctrl, vertical_clearance=vertical_clearance)

        msg = save_camera_view(
            bridge,
            out_dir / f'image_car{car_i}.png',
            'B1/rrbot/camera1/image_raw'
        )
        msg.header.frame_id = f"{car_i}"

        pub_clueboards.publish(msg)

    ctrl.increase_velocity((0, 0, 100)) # Send the robot to heaven

    rospy.spin()
