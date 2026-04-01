#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from numpy.typing import NDArray
from cv_bridge import CvBridge
from scipy.signal import fftconvolve
from std_srvs.srv import Empty

DEFAULT_QUEUE_SIZE = 10

class CrossCorrelationNode:
    """
    Computes the cross corellation of an image topic between t0 and t,
    To track motion over time.
    """

    def __init__(self, queue_size=DEFAULT_QUEUE_SIZE, debug_mode: bool = False):
        """
        Create a CrossCorrelationNode

        :param int queue_size: Queue size for all publishers and subscribers
        :param bool debug_mode: If True, also outputs a frame with the peak cross-correlation circled
        """
        self.name = "cross_correlation_node"
        self._queue_size = queue_size

        rospy.init_node(self.name, anonymous=True)

        self._camera_topic = "/B1/rrbot/camera_down/image_raw"
        self._camera_sub = rospy.Subscriber(self._camera_topic, Image, self.callback, queue_size=self._queue_size)

        self._xcorr_topic = "/B1/rrbot/camera_down/image_xcorr"
        self._xcorr_pub = rospy.Publisher(self._xcorr_topic, Image, queue_size=self._queue_size)

        if debug_mode:
            self._debug_topic = "/B1/rrbot/camera_down/image_xcorr_debug"
            self._debug_pub = rospy.Publisher(self._debug_topic, Image, queue_size=self._queue_size)
        else:
            self._debug_topic = None
            self._debug_pub = None

        self._bridge = CvBridge()

        # Reference image is the image with which the camera feed gets cross-correlated
        self.reference_img = None
        update_ref_serv = rospy.Service('update_xcorr_ref', Empty, self.update_reference_img)

    def run(self):
        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()

    def _on_shutdown(self):
        rospy.loginfo(f"{self.name} shutting down.")
    
    def callback(self, img: Image) -> None:
        try:
            self.process_image(img)
        except Exception as e:
            rospy.logerr(f"{self.name} failed to process image: {e}")

    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        """Convert to float32, collapse channels, and zero-mean."""
        img = img.astype(np.float32)
        if img.ndim == 3:
            img = img.mean(axis=2)
        return img - img.mean()
    
    def process_image(self, img: Image) -> None:
        cv_image = self._bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        processed = self._preprocess(cv_image)

        if self.reference_img is None:
            self.reference_img = processed

        xcorr_img = fftconvolve(processed, np.flip(self.reference_img), mode="same")
        xcorr_img = xcorr_img.astype(np.float32)

        xcorr_img_normalized = cv2.normalize(xcorr_img, None, 0, 255, cv2.NORM_MINMAX)
        xcorr_img_normalized = xcorr_img_normalized.astype(np.uint8)
        xcorr_img_msg = self._bridge.cv2_to_imgmsg(xcorr_img_normalized, encoding="mono8")
        self._xcorr_pub.publish(xcorr_img_msg)
        
        if self._debug_pub is None:
            return

        xcorr_img_normalized = xcorr_img_normalized.astype(np.uint8)
        _, _, _, peak = cv2.minMaxLoc(xcorr_img_normalized)
        vis = cv2.cvtColor(xcorr_img_normalized, cv2.COLOR_GRAY2BGR)
        cv2.circle(vis, center=peak, radius=20, color=(0, 0, 255), thickness=2)
        debug_img_msg: Image = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        self._debug_pub.publish(debug_img_msg)

    def update_reference_img(self, request):
        self.reference_img = None
        return []  # Empty service response

if __name__ == "__main__":
    node = CrossCorrelationNode(debug_mode=True)
    node.run()
