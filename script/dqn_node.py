#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_srvs import Empty
import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
import logging
import numpy as np

bridge = CvBridge()

logger = logging.getLogger(__name__)

class StageEnvironment(object):
  def __init__(self, max_random_start,
               observation_dims, data_format):
    self.env = gym.make(env_name)

    self.max_random_start = max_random_start
    self.action_size = 27

    self.data_format = data_format
    self.observation_dims = observation_dims

    self.screen = np.zeros((observation_dims,observation_dims,1), np.uint8)
    rospy.wait_for_service('reset_positions')
    self.resetStage = rospy.ServiceProxy('reset_positions', Empty)
  

  def new_game(self):
    self.resetStage()
    return self.preprocess(), 0, False

  def new_random_game(self):
    return self.new_game()

  def step(self, action, is_training=False):
    observation, reward, terminal, info = self.env.step(action)
    if self.display: self.env.render()
    return self.preprocess(observation), reward, terminal, info

  def preprocess(self):
    return self.screen

  def laserCB(self, data):
    try:
      self.screen = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow("Scree", screen)
    cv2.waitKey(3)

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

if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('dqn_node')

  # Subscribers:
  rospy.Subscriber('bridge/laser_image', Image, laserCB)
  
  # spin to keep the script for exiting
  rospy.spin()
