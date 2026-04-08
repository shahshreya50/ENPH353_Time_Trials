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
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
import cv2

# e.g. 'car0' -> (x, y, z, phi)
# assumes drone is in starting position
MODEL_POSITIONS: dict = {
    'B1_spawnpos': np.array([5.5, 2.5, 0.2, -1.5707963267948968]),
    'B1_grounded': np.array([5.500004840037357, 2.4999946488568625, 0.06429105580640433, -1.5700318536518063]),
    'car0': np.array([5.81, 1.64, 0.04, 0.0]),
    'car1': np.array([5.16, -1.35, 0.04, 0.0]),
    'car2': np.array([4.0, -1.67, 0.04, -1.57]),
    'car3': np.array([0.83, -0.54, 0.04, 3.1399999999999753]),
    'car4': np.array([0.83, 1.5, 0.04, 0.0]),
    'car5': np.array([-3.41, 1.71, 0.04, -1.57]),
    'car6': np.array([-3.8, -2.01, 0.04, 1.57]),
    'car7': np.array([-0.9, -1.2, 1.86, 1.57]),
}

# Angles phi depend on where the normal vector of is on the model
# For clue boards: normal vector is on the left when facing the sign
# For the drone: normal vector is forward at spawn

def get_offset(angle_rads: float, distance: float = 0.4):
    """
    Determine the displacement from a sign
    to where the drone should view it from
    """
    return np.array([
        distance * np.cos(angle_rads),
        distance * np.sin(angle_rads)
    ])

def fly_to_carx(x: int,
                ctrl: ForceController,
                drone_pos: np.ndarray = None,
                vertical_clearance: float = 1.0,
                end_height_above: float = 0.1):

    if drone_pos is None:
        # Default: spawn point
        drone_pos: np.ndarray = MODEL_POSITIONS['B1_spawnpos']            

    car_name = f"car{x}"

    # Fix the car theta so that 0 deg means the text faces +x
    car_pos: np.ndarray = MODEL_POSITIONS[car_name] + np.array([0, 0, 0, np.pi/2])
    

    # We want to align the rear of the drone with the normal of the sign face
    drone_rear_theta = drone_pos[3] + np.pi

    delta_pos = car_pos - drone_pos
    delta_xy = delta_pos[:2]
    delta_z = delta_pos[2]
    delta_phi = car_pos[3] - drone_rear_theta

    xy_offset = get_offset(car_pos[3])

    ctrl.increase_position((0, 0, delta_z + vertical_clearance))
    ctrl.increase_position((*(delta_xy + xy_offset), 0))
    ctrl.increase_angle((0, 0, delta_phi))
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

def save_camera_view(bridge: CvBridge, save_path: Path, topic: str) -> None:
    """
    Saves a fresh picture from a ros topic to a file.
    Blocks until a new message arrives.
    """
    msg = wait_for_fresh_message(topic, Image, timeout=5.0)
    
    if msg is not None:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(str(save_path), cv_image)
    else:
        rospy.logerr("Failed to capture fresh image.")


def get_euler_from_quaternion(q):
    """Returns (roll, pitch, yaw)"""
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def liftoff(ctrl: ForceController, z_increase: float = 1.0):
    # 1. Enable gravity compensation and move up
    ctrl.zero_force(with_offset=True)
    ctrl.increase_position((0, 0, z_increase))
    
    # 2. Get state from IMU
    msg = wait_for_fresh_message('/B1/imu/data', Imu, timeout=2.0)
    if not msg: return

    # 3. Cancel all angular velocities (Roll, Pitch, Yaw)
    # This prevents the drone from drifting while we calculate the angle fix
    omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    ctrl.increase_angular_velocity(-omega)

    # 4. Correct Orientation
    # Refresh message to get the state after the velocity stop
    msg = wait_for_fresh_message('/B1/imu/data', Imu, timeout=1.0)
    r, p, y = get_euler_from_quaternion(msg.orientation)
    
    target_yaw = -math.pi / 2.0
    dy = math.atan2(math.sin(target_yaw - y), math.cos(target_yaw - y))
    
    # Apply pulses to bring roll/pitch to 0 and yaw to target
    # We use -r and -p because we want to subtract the current tilt
    ctrl.increase_angle((-r, -p, dy))


if __name__ == "__main__":

    rospy.init_node('follow_route')

    MODEL_MASS = 20.00
    MODEL_I_ZZ = 0.1
    A_GRAVITY = 9.8
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

    # Teleport back up, but this time without gravity
    # ctrl.zero_force(with_offset=True)
    # respawn_model('B1')

    # Example integration of liftoff (optional)
    liftoff(ctrl, z_increase=0.1)
    rospy.sleep(10)

    #start publishers
    pub_clueboards = rospy.Publisher('/clueboard_images', Image, queue_size=8)
    pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
    rate = rospy.Rate(2)
    pub_score.publish('Team6,abcde,0,START')


    for car_i in range(8):        

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
        msg.header.frame_id = "{i}"

        pub_clueboards.publish(msg)

    pub_score.publish('Team6,abcde,-1,END')
    
    respawn_model('B1')
