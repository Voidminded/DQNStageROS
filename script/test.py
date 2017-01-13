#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters

bridge = CvBridge()

# Create publisher
publisher = rospy.Publisher("test",String,queue_size=1)

# Define Timer callback
def callback(event):
  msg = String()
  msg.data = "salam"
  publisher.publish(msg)
  #rospy.loginfo("Salam")

# Read parameter
pub_period = rospy.get_param("~pub_period",10.0)

# Create timer
#rospy.Timer(rospy.Duration.from_sec(pub_period),callback)

def laserCB(data):
  try:
    screen = np.squeeze(bridge.imgmsg_to_cv2(data, "mono8"))
  except CvBridgeError as e:
    print(e)
  cv2.imshow("Scree", screen)
  # rospy.loginfo(" dim : %s", screen.shape)
  cv2.waitKey(3)

def syncedCB( scan, odom):
  msg = String()
  msg.data = "salam"
  publisher.publish(msg)

if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('test_node')

  # Subscribers:
  rospy.Subscriber('bridge/laser_image', Image, laserCB)

  sub_scan_ = message_filters.Subscriber('base_scan', LaserScan)
  sub_pose_ = message_filters.Subscriber('base_pose_ground_truth', Odometry)
  ts = message_filters.TimeSynchronizer( [sub_scan_, sub_pose_], 1)
  ts.registerCallback( syncedCB)
  
  # spin to keep the script for exiting
  rospy.spin()
