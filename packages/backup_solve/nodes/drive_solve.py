# Set up python
import tensorflow as tf
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
import sys
sys.path.append('/home/fizzer/ros_ws/src/time_trials/packages/team6_utils/nodes')
import read_clues

def spawn_position(position):

  msg = ModelState()
  msg.model_name = 'B1'

  msg.pose.position.x = position[0]
  msg.pose.position.y = position[1]
  msg.pose.position.z = position[2]
  msg.pose.orientation.x = position[3]
  msg.pose.orientation.y = position[4]
  msg.pose.orientation.z = position[5]
  msg.pose.orientation.w = position[6]

  rospy.wait_for_service('/gazebo/set_model_state')
  try:
      set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
      resp = set_state( msg )

  except rospy.ServiceException:
      print ("Service call failed")


position_pink1 = [0.45, 0, 0.04, 0, 0, 0.707, 0.707]
position_pink3 = [-3.95, -2.35, 0.04, 0, 0, 0, 1]

rospy.init_node('state_machine', anonymous=True)
CAMERA_TOPIC = "/B1/rrbot/camera1/image_raw"
bridge = CvBridge()

print("Initialization complete")

#State 1: Start timer
pub_vel = rospy.Publisher('/B1/cmd_vel', Twist, 
  queue_size=1)
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
rate = rospy.Rate(2)
move = Twist()

rospy.sleep(1)
pub_score.publish('Team6,abcde,0,START')

#State 2: read first clue
print("Waiting for first image")
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue1 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,1,{clue1}")

#State 3: drive to next clueboard
move.linear.x = 1
pub_vel.publish(move)
rospy.sleep(8)
move.linear.x = 0
pub_vel.publish(move)
rospy.sleep(1)

#read second clue
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue2 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,2,{clue2}")


#turn to see 3rd clue
move.linear.x = 1
pub_vel.publish(move)
rospy.sleep(1)
move.linear.x = 0
move.angular.z = -1
pub_vel.publish(move)
rospy.sleep(4)
move.angular.z = 0
pub_vel.publish(move)
rospy.sleep(1)
print("Reading clue 3")
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue3 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,3,{clue3}")
rospy.sleep(1)

spawn_position(position_pink1)
rospy.sleep(1)

move.linear.x = -1
pub_vel.publish(move)
rospy.sleep(4)
move.linear.x = 0
pub_vel.publish(move)
rospy.sleep(1)
print("Reading clue 4")
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue4 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,4,{clue4}")

move.linear.x = 2
pub_vel.publish(move)
rospy.sleep(4.5)
move.linear.x = 0
move.angular.z = -2
pub_vel.publish(move)
rospy.sleep(3)
move.angular.z = 0
pub_vel.publish(move)

rospy.sleep(1)
print("Reading clue 5")
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue5 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,5,{clue5}")
rospy.sleep(1)


# ABANDON CLUE 6!!!
# move.angular.z = -1
# pub_vel.publish(move)
# rospy.sleep(4)
# move.angular.z = 0
# pub_vel.publish(move)
# rospy.sleep(1)
# print("Reading clue 6")
# msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
# cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
# clue6 = read_clues.read_clueboard(cv_image)
# pub_score.publish(f"Team6,abcde,6,{clue6}")
# rospy.sleep(1)

spawn_position(position_pink3)
rospy.sleep(1)
move.angular.z = 1
pub_vel.publish(move)
rospy.sleep(2)
move.angular.z = 0
pub_vel.publish(move)
rospy.sleep(0.1)

print("Reading clue 7")
msg = rospy.wait_for_message(CAMERA_TOPIC, Image,)
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
clue7 = read_clues.read_clueboard(cv_image)
pub_score.publish(f"Team6,abcde,7,{clue7}")
rospy.sleep(1)

# Not worth doing Clue 8 unless we want to implement some kind of PID

rospy.sleep(1)
pub_score.publish(f"Team6,abcde,-1,yay")



