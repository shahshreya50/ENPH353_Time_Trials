#!/usr/bin/env python3

import rospy
import numpy as np
from controller import ForceController
from move_relative import respawn_model


DEG_TO_RAD = np.pi / 180

def run_course(ctrl: ForceController):
    # 1. Respawn the model at the start with zero force applied
    respawn_model('B1')
    ctrl.zero_force(with_offset=False)

    # rospy.sleep(2) # Let drone fall the ground

    # Cancel gravity
    ctrl.zero_force(with_offset=True)

    rospy.sleep(0.5)

    # x, y, z, theta
    moves = [
        # car4
        (0, 0, 1, 0),
        (-4.8, 0, 0, 0),
        (0, 0, -1, 0),
        (0, -0.7, 0, 0),

        # car5
        (0, 0.7, 0, 0),
        (0, 0, 0, -np.pi/2),
        (-3.7, 0, 0, 0),
        (0, -0.7, 0, 0),

        # car7
        (0, 0, 1.8, 0),
        (0, -3, 0, 0),
        (0, 0, 0, np.pi),
        (1.8, 0, 0, 0),

        # car6
        (0, -0.8, 0, 0),
        (-2.8, 0, 0, 0),
        (0, 0, -1.9, 0),
    ]

    for move in moves:

        linear = move[:3] if move[:3] != (0, 0, 0) else None
        phi_z = move[3] if move[3] != 0 else None

        if linear is not None:
            ctrl.increase_position(linear)
        if phi_z is not None:
            ctrl.increase_angle((0, 0, phi_z))
        rospy.sleep(0.2)


if __name__ == "__main__":

    # Disable wind to run this test! This can be done in the Gazebo GUI under B1 > chassis

    rospy.init_node('test_force_controller')

    model_mass = 20.00
    model_izz = 0.1
    a_gravity = 9.8
    ctrl = ForceController(
        model_mass,
        model_izz,
        force_offsets=(0, 0, model_mass * a_gravity),
        default_impulse_duration=0.2,
    )

    num_runs = 1
    for i in range(num_runs):
        run_course(ctrl)    
        rospy.sleep(0.5)
