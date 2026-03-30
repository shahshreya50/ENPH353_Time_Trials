#!/usr/bin/env python3

from sensor_msgs.msg import Image
from abc import ABC, abstractmethod
from cv_bridge import CvBridge
import rospy

DEFAULT_QUEUE_SIZE = 10

class ImageProcessingNode(ABC):
    """
    Base class for ROS Noetic / Gazebo image processing node.

    Represents a node that subscribes to a topic of type image,
    and maps images to images in another topic.

    All children must override `process_image`
    """

    def __init__(self, name: str, sub_topic: str, pub_topic: str, queue_size=DEFAULT_QUEUE_SIZE):
        self.name = name
        self._sub_topic = sub_topic
        self._pub_topic = pub_topic
        self._queue_size = queue_size

        self._init_node()
        self._pub = self._init_pub()
        self._sub = self._init_sub()
        self._bridge = CvBridge()

    def _init_node(self):
        rospy.init_node(self.name, anonymous=True)

    def _init_pub(self):
        publisher = rospy.Publisher(self._pub_topic, Image, queue_size=self._queue_size)
        return publisher
    
    def _init_sub(self):
        subscriber = rospy.Subscriber(self._sub_topic, Image, self.callback, queue_size=self._queue_size)
        return subscriber
    
    def run(self):
        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()

    def _on_shutdown(self):
        rospy.loginfo(f"{self.name} shutting down.")
    
    def callback(self, img: Image) -> None:
        try:
            result = self.process_image(img)
            self._pub.publish(result)
        except Exception as e:
            rospy.logerr(f"{self.name} failed to process image: {e}")

    def __repr__(self):
        return (f"{self.__class__.__name__}(name={self.name!r}, "
                f"sub={self._sub_topic!r}, pub={self._pub_topic!r})")
    
    @abstractmethod
    def process_image(self, img: Image) -> Image:
        pass
