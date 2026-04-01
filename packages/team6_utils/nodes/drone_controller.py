#!/usr/bin/env python3
"""@package docstring
Controller to enable navigation around the world and fight wind.
Date: 2026-03-31
Author: Jonah Lee
"""

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from std_srvs.srv import Empty
import cv2


DEFAULT_QUEUE_SIZE = 10


class DroneController:
    def __init__(
        self,
        xcorr_topic: str = "/B1/rrbot/camera_down/image_xcorr",
        force_control_topic: str = "/B1/drone_controller/cmd",
        queue_size: int = DEFAULT_QUEUE_SIZE,
        kp: float = 0.01,
        ki: float = 0.000,
        kd: float = 0.01,
    ):
        self._queue_size = queue_size
        rospy.init_node('drone_controller', anonymous=True)

        self._bridge = CvBridge()

        self._xcorr_sub = rospy.Subscriber(xcorr_topic, Image, self.process_xcorr_img, queue_size=self._queue_size)
        self._force_pub = rospy.Publisher(force_control_topic, Wrench, queue_size=self._queue_size)

        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # PID state — error is 2D (x, y)
        self._error: np.ndarray = np.zeros(2)
        self._integral: np.ndarray = np.zeros(2)
        self._prev_error: np.ndarray = np.zeros(2)
        self._last_time: float = None

        # Setpoint is the image centre — updated when update_setpoint() is called
        self._setpoint: np.ndarray = None

    def process_xcorr_img(self, img: Image) -> None:
        """Compute PID error from peak location in xcorr image and publish force."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

            # Find peak of cross-correlation
            _, _, _, peak = cv2.minMaxLoc(cv_image)
            peak = np.array(peak, dtype=float)  # (x, y) in image pixels

            # On first image, initialise setpoint to image centre
            if self._setpoint is None:
                h, w = cv_image.shape[:2]
                self._setpoint = np.array([w / 2.0, h / 2.0])

            now = rospy.get_time()
            dt = (now - self._last_time) if self._last_time is not None else 0.0
            self._last_time = now

            # Error: how far is the peak from the setpoint?
            # Assumes image x -> robot x, image y -> robot y (fix if needed)
            self._error = peak - self._setpoint
            if dt > 0:
                self._integral += self._error * dt
                derivative = (self._error - self._prev_error) / dt
            else:
                derivative = np.zeros(2)
            self._prev_error = self._error

            force = (self.kp * self._error
                   + self.ki * self._integral
                   + self.kd * derivative)

            msg = Wrench()
            msg.force.x = float(force[0])
            msg.force.y = float(force[1])
            msg.force.z = 0.0
            print("Force cmd (x,y):", msg.force.x, msg.force.y)
            self._force_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"drone_controller failed to process image: {e}")

    def update_setpoint(self) -> None:
        """Call update_xcorr_ref service to reset the cross-correlation reference,
        and reset PID state so the controller treats current position as the new setpoint."""
        rospy.wait_for_service('update_xcorr_ref', timeout=5.0)
        try:
            update_ref = rospy.ServiceProxy('update_xcorr_ref', Empty)
            update_ref()
            # Reset PID state
            self._integral = np.zeros(2)
            self._prev_error = np.zeros(2)
            self._setpoint = None  # will re-initialise on next image
            rospy.loginfo("drone_controller: setpoint updated.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"drone_controller: failed to update setpoint: {e}")

    def _on_shutdown(self):
        rospy.loginfo("drone_controller shutting down.")

    def run(self):
        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()


if __name__ == "__main__":
    node = DroneController()
    node.run()