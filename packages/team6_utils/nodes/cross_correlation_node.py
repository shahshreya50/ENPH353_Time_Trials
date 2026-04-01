#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.signal import fftconvolve
from std_srvs.srv import Empty
import threading

DEFAULT_QUEUE_SIZE = 10

class CrossCorrelationNode:
    """
    Computes the cross correlation of an image topic between t0 and t,
    to track motion over time.
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
        self._xcorr_topic = "/B1/rrbot/camera_down/image_xcorr"
        self._xcorr_pub = rospy.Publisher(self._xcorr_topic, Image, queue_size=1)  # queue_size=1: always publish latest

        if debug_mode:
            debug_topic = "/B1/rrbot/camera_down/image_xcorr_debug"
            self._debug_pub = rospy.Publisher(debug_topic, Image, queue_size=1)
            debug_raw_topic = "/B1/rrbot/camera_down/image_raw_debug"
            self._debug_raw_pub = rospy.Publisher(debug_raw_topic, Image, queue_size=1)
        else:
            self._debug_pub = None
            self._debug_raw_pub = None

        self._bridge = CvBridge()
        self.reference_img: np.ndarray = None
        self._lock = threading.Lock()
        self._pending_reset = False  # flag to drop frames until fresh image arrives

        rospy.Service('update_xcorr_ref', Empty, self.update_reference_img)

        # Subscribe last, after everything is initialised
        self._camera_sub = rospy.Subscriber(
            self._camera_topic, Image, self.callback, queue_size=1, buff_size=2**24  # queue_size=1: drop stale frames
        )

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

        with self._lock:
            if self._pending_reset:
                self.reference_img = processed
                self._pending_reset = False
                rospy.loginfo(f"{self.name}: reference image updated.")
            if self.reference_img is None:
                self.reference_img = processed
            reference = self.reference_img  # local copy so we can release lock

        xcorr_img = fftconvolve(processed, np.flip(reference), mode="same")
        xcorr_img = xcorr_img.astype(np.float32)

        xcorr_img_normalized = cv2.normalize(xcorr_img, None, 0, 255, cv2.NORM_MINMAX)
        xcorr_img_normalized = xcorr_img_normalized.astype(np.uint8)

        xcorr_img_msg = self._bridge.cv2_to_imgmsg(xcorr_img_normalized, encoding="mono8")
        self._xcorr_pub.publish(xcorr_img_msg)

        if self._debug_pub is None:
            return

        _, _, _, peak = cv2.minMaxLoc(xcorr_img_normalized)
        vis = cv2.cvtColor(xcorr_img_normalized, cv2.COLOR_GRAY2BGR)
        cv2.circle(vis, center=peak, radius=20, color=(255, 0, 0), thickness=2)
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(vis, encoding="rgb8"))

        # Draw peak location onto the original raw image
        raw_vis = self._bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        raw_vis = cv2.normalize(raw_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if raw_vis.ndim == 2:
            raw_vis = cv2.cvtColor(raw_vis, cv2.COLOR_GRAY2BGR)
        cv2.circle(raw_vis, center=peak, radius=20, color=(255, 0, 0), thickness=2)
        self._debug_raw_pub.publish(self._bridge.cv2_to_imgmsg(raw_vis, encoding="rgb8"))

    def update_reference_img(self, request):
        with self._lock:
            self._pending_reset = True
        return []

if __name__ == "__main__":
    node = CrossCorrelationNode(debug_mode=True)
    node.run()