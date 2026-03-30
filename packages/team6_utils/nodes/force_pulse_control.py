#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Wrench
import sys
import select
import termios
import tty
from move_relative import respawn_model

msg = """
Control Forces with Keyboard
---------------------------
w/s : +x / -x force pulse
a/d : +y / -y force pulse
r/f : +z / -z force pulse (above)

j/l : +z / -z torque pulse

space : reset forces
p : respawn
q : quit
"""

force_pulse = 1.0
torque_pulse = 0.01

force_z_step = 0.005

force_pulse_length = 1
torque_pulse_length = 1


drone_mass = 20.22
gravity = 9.8
gravity_force = drone_mass * gravity


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def get_default_wrench():
    wrench = Wrench()
    wrench.force.z = gravity_force # Counteract gravity
    return wrench


def main():

    MODEL_NAME = 'B1'

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("keyboard_force_pulse_controller")

    topic = rospy.get_param("~topic", f"/{MODEL_NAME}/cmd_force")
    pub = rospy.Publisher(topic, Wrench, queue_size=1)
    force_ticks_remaining = 0
    torque_ticks_remaining = 0
    wrench = get_default_wrench()

    print(msg)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == 'w':
                wrench.force.y = -force_pulse
                force_ticks_remaining = force_pulse_length
            elif key == 's':
                wrench.force.y = force_pulse
                force_ticks_remaining = force_pulse_length
            elif key == 'a':
                wrench.force.x = force_pulse
                force_ticks_remaining = force_pulse_length
            elif key == 'd':
                wrench.force.x = -force_pulse
                force_ticks_remaining = force_pulse_length
            elif key == 'r':
                wrench.force.z = gravity_force + force_pulse
                force_ticks_remaining = force_pulse_length
            elif key == 'f':
                wrench.force.z = gravity_force - force_pulse
                force_ticks_remaining = force_pulse_length

            # elif key == 'i':
            #     wrench.torque.x += torque_pulse
            # elif key == 'k':
            #     wrench.torque.x -= torque_pulse
            # elif key == 'j':
            #     wrench.torque.y += torque_pulse
            # elif key == 'l':
            #     wrench.torque.y -= torque_pulse
            elif key == 'j':
                wrench.torque.z = torque_pulse
                torque_ticks_remaining = torque_pulse_length
            elif key == 'l':
                wrench.torque.z = -torque_pulse
                torque_ticks_remaining = torque_pulse_length

            elif key == ' ':
                wrench = get_default_wrench()
            
            elif key == 'p':
                wrench = get_default_wrench()
                respawn_model(MODEL_NAME)

            elif key == 'q':
                break


            if force_ticks_remaining == 0:
                wrench.force.x = 0
                wrench.force.y = 0
                wrench.force.z = gravity_force
            else:
                force_ticks_remaining -= 1

            if torque_ticks_remaining == 0:
                wrench.torque.x = 0
                wrench.torque.y = 0
                wrench.torque.z = 0
            else:
                torque_ticks_remaining -= 1

            pub.publish(wrench)

            print(
                f"\rForce: ({wrench.force.x:.2f}, {wrench.force.y:.2f}, {wrench.force.z:.2f}) "
                f"Torque: ({wrench.torque.x:.2f}, {wrench.torque.y:.2f}, {wrench.torque.z:.2f})",
                end=""
            )

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()