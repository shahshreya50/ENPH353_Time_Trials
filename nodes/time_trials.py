#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rospy.init_node('topic_publisher')
pub_vel = rospy.Publisher('/B1/cmd_vel', Twist, 
  queue_size=1)
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.0

rospy.sleep(1)
pub_score.publish('TeamRed,multi21,0,JEDIS')
pub_vel.publish(move)

rospy.sleep(10)
pub_score.publish("benoit_blanc,111,-1,a")
move.linear.x = 0.0
pub_vel.publish(move)