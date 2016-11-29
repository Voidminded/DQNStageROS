#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
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
    
    #####################
    # Followings are not used yet
    self.syncLaser = False
    self.syncPose = False
    self.syncGoal = False
    self.syncDir = False

    self.sentTime = 0
    #####################

    self.pose.x = 0.0
    self.pose.y = 0.0
    self.prevPose.x = 0.0
    self.prevPose.y = 0.0
    self.goal.x = 0.0
    self.goal.y = 0.0
    self.dir = 0.0
    self.prevDist = 0.0
    self.terminal =False

    rospy.wait_for_service('reset_positions')
    self.resetStage = rospy.ServiceProxy('reset_positions', Empty)

    self.pub_action_ = rospy.Publisher("dqn/selected_action",Int8,queue_size=1)
    
    # Subscribers:
    rospy.Subscriber('bridge/laser_image', Image, self.laserCB)
    rospy.Subscriber('bridge/current_dir', Float32, self.directionCB)
    rospy.Subscriber('bridge/impact', Bool, self.impactCB)
    rospy.Subscriber('bridge/current_pose', Pose, self.positionCB)
    rospy.Subscriber('bridge/goal_pose', Pose, self.targetCB)

  
  
  def new_game(self):
    self.resetStage()
    self.terminal = False
    cv2.waitKey(30)
    return self.preprocess(), 0, False

  def new_random_game(self):
    # TODO: maybe start from a random position not just reset the robot's position to initial point
    # which needs some edit in stage_ros
    return self.new_game()

  def step(self, action, is_training=False):
    self.prevPose.x = self.pose.x
    self.prevPose.y = self.pose.y

    if action == -1:
      # Step with random action
      action = int(random.random()*(self.action_size+1))

    msg = Int8()
    msg.data = action
    self.pub_action_.publish( msg)

    cv2.waitKey(9)

    dist = (self.pose.x - self.prevPose.x)**2 + (self.pose.y - self.prevPose.y)**2
    reward = self.prevDist - dist
    self.prevDist = dist


    observation, reward, terminal, info = self.env.step(action)
    return self.preprocess(observation), reward, terminal, info

  def preprocess(self):
    return self.screen

  def laserCB(self, data):
    try:
      self.screen = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow("Screen", screen)

  def directionCB(self, data):
    self.dir = data

  def positionCB( self, data):
    self.pose.x = data.position.x
    self.pose.y = data.position.y

  def targetCB( self, data):
    self.gaol.x = data.position.x
    self.goal.y = data.position.y

  def impactCB( self, data):
    self.terminal = data

if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('dqn_node')

  # spin to keep the script for exiting
  rospy.spin()
