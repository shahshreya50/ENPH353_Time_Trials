#!/usr/bin/env python3

import cv2
import numpy as np
from sensor_msgs.msg import Image
from numpy.typing import NDArray
from scipy.signal import fftconvolve
from image_processing_node import ImageProcessingNode

class CrossCorrelationNode(ImageProcessingNode):
    """
    Computes the cross corellation of an image topic between t0 and t,
    To track motion over time.
    """

    def __init__(self):
        super().__init__(
            name="cross_correlation_node",
            sub_topic="/B1/rrbot/camera_down/image_raw",
            pub_topic="/B1/rrbot/camera_down/image_xcorr",
            queue_size=10
        )

        self.reference_img = None # TODO: type hint

    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        """Convert to float32, collapse channels, and zero-mean."""
        img = img.astype(np.float32)
        if img.ndim == 3:
            img = img.mean(axis=2)
        return img - img.mean()

    def process_image(self, img: Image) -> Image:
        cv_image = self._bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        processed = self._preprocess(cv_image)

        if self.reference_img is None:
            self.reference_img = processed

        res = fftconvolve(processed, np.flip(self.reference_img), mode="same")
        res = res.astype(np.float32)

        res_normalized = cv2.normalize(res, None, 0, 255, cv2.NORM_MINMAX)
        res_normalized = res_normalized.astype(np.uint8)

        _, _, _, peak = cv2.minMaxLoc(res_normalized)
        vis = cv2.cvtColor(res_normalized, cv2.COLOR_GRAY2BGR)
        cv2.circle(vis, center=peak, radius=20, color=(0, 0, 255), thickness=2)

        return self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")


if __name__ == "__main__":
    node = CrossCorrelationNode()
    node.run()
