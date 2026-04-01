#!/usr/bin/env python3
from geometry_msgs.msg import Wrench
from move_relative import respawn_model, move_model_relative, DEFAULT_ORIENTATION, DEFAULT_WORLD_POSE
import rospy
import time
from std_srvs.srv import Empty

def update_setpoint():
    rospy.wait_for_service('update_xcorr_ref')  # blocks until service is available
    try:
        update_ref = rospy.ServiceProxy('update_xcorr_ref', Empty)
        update_ref()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

drone_mass = 20.22
gravity = 9.8
gravity_force = drone_mass * gravity

upper_pose = DEFAULT_WORLD_POSE
upper_pose.position.z += 1

scale = 10

def send_control(cmd_wrench: Wrench):
    # print("CMD Wrench (x,y):", cmd_wrench.force.x, cmd_wrench.force.y)
    new_wrench = Wrench()
    new_wrench.force.x = -cmd_wrench.force.x * scale
    new_wrench.force.y = cmd_wrench.force.y * scale
    new_wrench.force.z = gravity_force
    # print("Wrench (x,y,z):", new_wrench.force.x, new_wrench.force.y, new_wrench.force.z)
    force_pub.publish(new_wrench)

if __name__ == "__main__":

    """
    Teleports the drone above the starting position,
    then uses PID control from drone_controller to keep the drone from moving in the wind.

    Requires cross_correlation_node and drone_controller_node to be running
    """

    rospy.init_node('wind_control_test')
    
    force_pub = rospy.Publisher("B1/cmd_force", Wrench, queue_size=10)

    # Loop to 'wake up' the cmd_force topic
    t0 = time.time()
    while time.time() - t0 < 1:
        wrench = Wrench()
        force_pub.publish(wrench)

    wrench = Wrench()
    wrench.force.z = gravity_force
    force_pub.publish(wrench)

    move_model_relative('B1', upper_pose)

    time.sleep(0.5)
    update_setpoint()

    cmd_sub = rospy.Subscriber("/B1/drone_controller/cmd", Wrench, send_control, queue_size=10)
    rospy.spin()