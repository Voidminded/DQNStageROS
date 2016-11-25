#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Create publisher
#publisher = rospy.Publisher("~topic",String,queue_size=1)

# Define Timer callback
def callback(event):
  #msg = String()
  #msg.data = "%s is %s!" %(util.getName(),util.getStatus())
  #publisher.publish(msg)
  rospy.loginfo("Salam")

# Read parameter
pub_period = rospy.get_param("~pub_period",10.0)

# Create timer
#rospy.Timer(rospy.Duration.from_sec(pub_period),callback)

def laserCB(data):
  try:
    screen = bridge.imgmsg_to_cv2(data, "mono8")
  except CvBridgeError as e:
    print(e)
  cv2.imshow("Scree", screen)
  cv2.waitKey(3)

if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('dqn_node')

  # Subscribers:
  rospy.Subscriber('bridge/laser_image', Image, laserCB)
  
  # spin to keep the script for exiting
  rospy.spin()
