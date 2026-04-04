#!/usr/bin/env python3
from geometry_msgs.msg import Wrench
import rospy
from typing import Collection
import numpy as np


"""
GazeboROSForce controller utility

docs:

    "This plugin collects data from a ROS topic and applies wrench to a link accordingly.
    The last received force will be continuously added to the link at every simulation iteration.
    Send an empty / zero message to stop applying a force."

Ensure `rospraram get /use_sim_time` is set to true, so rospy sleep works in sim time.

"""


class ForceController:
    """
    Drone force controller with force pulse and velocity pulse methods

    This is not its own node, so rospy.init_node() must be called elsewhere.
    """
    def __init__(
            self,
            model_mass: float,
            force_offsets: tuple = None,
            default_impulse_duration: float = None,
            default_travel_duration: float = None
        ):

        self.mass = model_mass

        if force_offsets is None:
            self.force_offsets = (0, 0, 0)
        else:
            self.force_offsets = force_offsets

        self.default_impulse_duration = default_impulse_duration if default_impulse_duration else 0.1
        self.default_travel_duration = default_travel_duration if default_travel_duration else 0.1

        self._pub = rospy.Publisher('/B1/cmd_force', Wrench, queue_size=1)
        
        # Wait for subscriber to connect
        rospy.sleep(0.5)

        self.zero_force()
    
    def send_wrench(self, with_offset=True) -> None:
        """
        Apply the Wrench self.wrench
        """
        if not with_offset:
            self._pub.publish(self.wrench)
            return

        offset_wrench = Wrench()
        offset_wrench.force.x = self.wrench.force.x + self.force_offsets[0]
        offset_wrench.force.y = self.wrench.force.y + self.force_offsets[1]
        offset_wrench.force.z = self.wrench.force.z + self.force_offsets[2]
        offset_wrench.torque = self.wrench.torque
        self._pub.publish(offset_wrench)

    def zero_force(self, with_offset=True) -> None:
        self.wrench = Wrench()
        self.send_wrench(with_offset)

    def apply_force(self, fx: float, fy: float, fz: float) -> None:
        """Apply a force with no torque. Updates self.wrench."""
        self.wrench = Wrench()
        self.wrench.force.x = fx
        self.wrench.force.y = fy
        self.wrench.force.z = fz
        self.send_wrench()

    def apply_torque(self, tx: float, ty: float, tz: float) -> None:
        """Apply a torque with no force. Updates self.wrench."""
        self.wrench = Wrench()
        self.wrench.torque.x = tx
        self.wrench.torque.y = ty
        self.wrench.torque.z = tz
        self.send_wrench()

    def apply_force_pulse(self, force: Collection, duration: float = None) -> None:
        """
        Apply a force with magnitude and direction `(force[0], force[1], force[2])`
        for `duration` seconds. Does not apply torque.
        """
        if not isinstance(force, np.ndarray):
            force = np.array([fi for fi in force])

        if duration is None:
            duration = self.default_impulse_duration

        self.apply_force(*force)
        rospy.sleep(duration)
        self.zero_force()

    def increase_velocity(self, delta_v: Collection, duration: float = None) -> None:
        """
        Use knowledge of the model mass to create an impulse which imparts a known change to velocity

        F = m dv/dt
        for const F,
        F*t = m*delta_V
        F = (m/t)*delta_V
        """
        if not isinstance(delta_v, np.ndarray):
            delta_v = np.array([vi for vi in delta_v])

        if duration is None:
            duration = self.default_impulse_duration

        force: np.ndarray = self.mass / duration * delta_v
        self.apply_force_pulse(force, duration)

    def increase_position(self, delta_pos: Collection, impulse_duration: float = None, travel_duration: float = None) -> None:
        """
        Use knowledge of the model mass to create a pair of opposite impulses which impart a known change to position

        Does not account for initial velocity of model: if this takes 1s and v0 is 1m/s, it will travel an additional meter.

        x2 - x1 = (1/2 * a1 * t1^2) + (v2 * t2) + (v2 * t3 - 1/2 * |a3| t3^2)
        x2 - x1 = 1/2 * a * impulse_duration**2 + (a * impulse_duration) * travel_duration + ((a * impulse_duration) * impulse_duration - 1/2 * a * impulse_duration**2)
        where a = F/m.

        delta_x = a * (1/2 * impulse_duration**2 + impulse_duration*travel_duration + impulse_duration**2 - 1/2 * impulse_duration**2)
                = a * (impulse_duration**2 + impulse_duration*travel_duration)

        F = m * (delta_x / (impulse_duration**2 + impulse_duration*travel_duration))
        """
        if not isinstance(delta_pos, np.ndarray):
            delta_pos = np.array([xi for xi in delta_pos])
        
        if impulse_duration is None:
            impulse_duration = self.default_impulse_duration
        
        if travel_duration is None:
            travel_duration = self.default_travel_duration

        force: np.ndarray = self.mass / (impulse_duration ** 2 + impulse_duration * travel_duration) * delta_pos

        self.apply_force_pulse(force, impulse_duration)
        rospy.sleep(travel_duration)
        self.apply_force_pulse(-force, impulse_duration)


if __name__ == "__main__":

    # Disable wind to run this test! This can be done in the Gazebo GUI under B1 > chassis

    from gazebo_msgs.srv import GetModelState, SetModelState
    from gazebo_msgs.msg import ModelState
    from move_relative import respawn_model

    def get_model_state(model_name: str) -> ModelState:
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state: ModelState = get_state(model_name, 'world')
            return model_state
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return ModelState()    

    rospy.init_node('test_force_controller')

    model_mass = 20.22
    a_gravity = 9.8
    ctrl = ForceController(
        model_mass,
        force_offsets=(0, 0, model_mass * a_gravity),
        default_impulse_duration=0.1,
        default_travel_duration=0.1
    )

    # 1. Respawn the model at the start with zero force applied
    respawn_model('B1')
    ctrl.zero_force(with_offset=False)

    # rospy.sleep(2) # Let drone fall the ground

    # Cancel gravity
    ctrl.zero_force(with_offset=True)

    # Move drone up and towards the center of the map
    ctrl.increase_position((-3, -3, 1))

    rospy.sleep(2)

    moves = [
        (1, 0, 0),
        (0, 1, 0),
        (0, 0, 1),
        (-1, 0, 0),
        (0, -1, 0),
        (0, 0, -1),
    ]

    while True:
        for move in moves:
            ctrl.increase_position(move, impulse_duration=0.01, travel_duration=0.05)
