#!/usr/bin/env python3
import rospy
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Point, Twist


DEFAULT_ORIENTATION = Quaternion(0.0, 0.0, -np.sqrt(2)/2, np.sqrt(2)/2)


# e.g. 'car0' -> (x, y, z, phi)
# assumes drone is in starting position
VIEW_POSITIONS: dict = {
    'car0': np.array([5.61, 2.14, 0.1, -np.pi/2+0.4]),
    'car1': np.array([5.46, -0.85, 0.1, -np.pi/2-0.45]),
    'car2': np.array([4.5, -1.67, 0.1, np.pi]),
    'car3': np.array([0.43, -1.0, 0.1, np.pi/2-0.6]),
    'car4': np.array([0.53, 2.0, 0.1, -np.pi/2+0.5]),
    'car5': np.array([-3.01, 1.71, 0.1, np.pi]),
    'car6': np.array([-4.4, -2.01, 0.3, 0]),
    'car7': np.array([-1.5, -1.2, 1.93, 0]),
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

def teleport_to_carx(x: int):

    car_name = f"car{x}"

    # Fix the car theta so that 0 deg means the text faces +x
    car_pos: np.ndarray = VIEW_POSITIONS[car_name]

    state = ModelState()
    state.model_name = 'B1'
    state.pose.position = Point(car_pos[0], car_pos[1], car_pos[2])
    theta = car_pos[3]
    state.pose.orientation.x = 0
    state.pose.orientation.y = 0
    state.pose.orientation.z = np.sin(theta/2)
    state.pose.orientation.w = np.cos(theta/2)
    state.twist = Twist()
    state.reference_frame = "world"

    # Apply the movement in Gazebo
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_state(state)

    

if __name__ == "__main__":

    teleport_to_carx(7)