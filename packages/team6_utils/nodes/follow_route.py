#!/usr/bin/env python3
import rospy
import numpy as np
from controller import ForceController
from move_relative import respawn_model


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


def get_offset(angle_rads: float, distance: float = 0.3):
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


if __name__ == "__main__":

    # Disable wind to run this test! This can be done in the Gazebo GUI under B1 > chassis

    rospy.init_node('test_force_controller')

    MODEL_MASS = 20.00
    MODEL_I_ZZ = 0.1
    A_GRAVITY = 9.8
    ctrl = ForceController(
        MODEL_MASS,
        MODEL_I_ZZ,
        force_offsets=(0, 0, MODEL_MASS * A_GRAVITY),
        default_impulse_duration=0.02,
    )

    # ==========================================================
    # Simulate a gazebo reset
    # ==========================================================

    # Respawn the model at the start with zero force applied
    # This makes the starting state more realistic because this
    # node will start a bit after the Gazebo simulation
    respawn_model('B1')
    ctrl.zero_force(with_offset=False)
    rospy.sleep(2)

    # ==========================================================
    # This is the point from which we assume the node will start
    # ==========================================================

    # Teleport back up, but this time without gravity
    ctrl.zero_force(with_offset=True)
    respawn_model('B1')

    while(True):
        for car_i in range(8):

            rospy.sleep(1)

            vertical_clearance = 4 if car_i == 6 else 1

            respawn_model('B1')
            rospy.sleep(1)
            fly_to_carx(car_i, ctrl, vertical_clearance=vertical_clearance)
