#!/usr/bin/env python3
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy
from cv_bridge import CvBridge
import read_clues



rospy.init_node('text_extraction', anonymous=True)
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
rate = rospy.Rate(2)

bridge = CvBridge()


def callback(msg):
    index = int(msg.header.frame_id) + 1
    print(f"Reading image {index}")
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    clue = read_clues.read_clueboard(cv_image)
    pub_score.publish(f"Team6,abcde,{index},{clue}")
    print("Success")

    if index >= 8:
        rospy.sleep(0.1)
        pub_score.publish('Team6,abcde,-1,END')

rospy.Subscriber("/clueboard_images", Image, callback)
rospy.spin()


