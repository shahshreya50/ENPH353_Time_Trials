#! /usr/bin/env python3

"""@package docstring
Describes ImageSaver, a class which wraps a subscriber to an image topic and
provides a function to block, wait for a new image message, and save it.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from pathlib import Path
from datetime import datetime

class ImageSaver:
    """Wrapper around an image subscriber to provide blocking image save functionality"""
    def __init__(
        self,
        image_sub_topic: str,
        out_dir: Path = None,
        bridge: CvBridge = None,
        skip_sleep: bool = False,
        timeout: float = 1.0,
        num_discards: int = 0,
    ):

        # Set up the output file path
        if out_dir is None:
            out_dir = Path.cwd()

        self.out_dir: Path = out_dir
        self.img_i: int = 0
        self.filename: str = None
        self.filepath: Path = None
        self.image_sub: rospy.Subscriber = rospy.Subscriber(image_sub_topic, Image, self.callback, queue_size=1)
        self.bridge: CvBridge = bridge if bridge is not None else CvBridge()
        self.timeout: float = timeout
        self.num_discards: int = num_discards

        # Flag to indicate when we want to save an image
        self._image_requested: bool = False
        # Number of images to discard before saving
        self._discards_remaining: int = 0

        if not skip_sleep:
            # Wait for image_sub to get running
            rospy.sleep(1)

    def find_unique_filename(self):
        filename_raw = f"{self.image_sub.name}_{self.img_i}.png"
        filename_clean = filename_raw.replace('/', '_')
        while Path.exists(self.out_dir / filename_clean):
            self.img_i += 1
            filename_raw = f"{self.image_sub.name}_{self.img_i}.png"
            filename_clean = filename_raw.replace('/', '_')
        return filename_clean

    def save_image_blocking(self):
        """
        Save the latest image from the ImageSaver's subcribed topic.
        Blocks until completion, with a timeout after `self.timeout` seconds.
        """
        self._discards_remaining = self.num_discards
        self._image_requested = True

        t0 = rospy.get_time()

        # Wait for image
        while rospy.get_time() - t0 < self.timeout:

            if not self._image_requested:
                print(f"Saved image {self.filename} to {self.out_dir}!")
                print(f"Time taken: {rospy.get_time() - t0:.4f}")
                return

            rospy.sleep(0.01)
        
        print(f"Failed to save image! Timed out after {self.timeout:.4f} seconds")
        return

    def callback(self, data: Image):
        if not self._image_requested:
            return
        
        if self._discards_remaining > 0:
            self._discards_remaining -= 1
            # Skip this image
            return

        # Image save requested!
        try:
            image_to_save = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Update the filename
            self.filename: str = self.find_unique_filename()
            self.filepath_str: str = str(self.out_dir / self.filename)

            print(f"{self.filepath_str}")
            self.image
            cv2.imwrite(self.filepath_str, image_to_save)

            # Remove flag after image request is satisfied
            self._image_requested = False

        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":

    rospy.init_node('image_saver_test')

    root: Path = Path(__file__).parent.parent.parent.parent
    data_dir: Path = root / 'data'

    timestamp = datetime.now().strftime(r"%d_%m_%Y_%H_%M_%S")
    out_dir_name = "test_outputs_" + timestamp
    out_dir: Path = data_dir / out_dir_name

    os.makedirs(out_dir, exist_ok=True)
    
    img_saver: ImageSaver = ImageSaver(
        '/B1/rrbot/camera1/image_raw', out_dir
    )

    rospy.sleep(1)

    img_saver.save_image_blocking()
