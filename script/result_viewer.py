#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def callback(data):
  if data.data == 10101.963:
    rospy.logwarn("Yeay ! reached the target :-)")
  else:
    rospy.loginfo("Last episode reward: %g", data.data)  

if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('result_node')

  # Subscribers:
  rospy.Subscriber('lastreward', Float64, callback, queue_size = 3)

  rospy.spin()
