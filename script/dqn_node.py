#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty as EmptySrv

import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
import logging
import numpy as np

import tensorflow as tf
from utils import get_model_dir
from networks.cnn import CNN
from networks.mlp import MLPSmall
from agents.statistic import Statistic

bridge = CvBridge()

logger = logging.getLogger(__name__)

flags = tf.app.flags

# Deep q Network
flags.DEFINE_boolean('use_gpu', True, 'Whether to use gpu or not. gpu use NHWC and gpu use NCHW for data_format')
flags.DEFINE_string('agent_type', 'DQN', 'The type of agent [DQN]')
flags.DEFINE_boolean('double_q', True, 'Whether to use double Q-learning')
flags.DEFINE_string('network_header_type', 'nature', 'The type of network header [mlp, nature, nips]')
flags.DEFINE_string('network_output_type', 'normal', 'The type of network output [normal, dueling]')

# Environment
flags.DEFINE_string('env_name', 'Stage', 'The name of gym environment to use')
flags.DEFINE_integer('max_random_start', 30, 'The maximum number of NOOP actions at the beginning of an episode')
flags.DEFINE_integer('history_length', 4, 'The length of history of observation to use as an input to DQN')
flags.DEFINE_integer('max_r', +9000, 'The maximum value of clipped reward')
flags.DEFINE_integer('min_r', -9000, 'The minimum value of clipped reward')
flags.DEFINE_string('observation_dims', '[80, 80]', 'The dimension of gym observation')
flags.DEFINE_boolean('random_start', True, 'Whether to start with random state')

# Training
flags.DEFINE_boolean('is_train', True, 'Whether to do training or testing')
flags.DEFINE_integer('max_delta', None, 'The maximum value of delta')
flags.DEFINE_integer('min_delta', None, 'The minimum value of delta')
flags.DEFINE_float('ep_start', 1., 'The value of epsilon at start in e-greedy')
flags.DEFINE_float('ep_end', 0.01, 'The value of epsilnon at the end in e-greedy')
flags.DEFINE_integer('batch_size', 32, 'The size of batch for minibatch training')
flags.DEFINE_integer('max_grad_norm', None, 'The maximum norm of gradient while updating')
flags.DEFINE_float('discount_r', 0.99, 'The discount factor for reward')

# Timer
flags.DEFINE_integer('t_train_freq', 4, '')

# Below numbers will be multiplied by scale
flags.DEFINE_integer('scale', 10000, 'The scale for big numbers')
flags.DEFINE_integer('memory_size', 100, 'The size of experience memory (*= scale)')
flags.DEFINE_integer('t_target_q_update_freq', 1, 'The frequency of target network to be updated (*= scale)')
flags.DEFINE_integer('t_test', 1, 'The maximum number of t while training (*= scale)')
flags.DEFINE_integer('t_ep_end', 100, 'The time when epsilon reach ep_end (*= scale)')
flags.DEFINE_integer('t_train_max', 5000, 'The maximum number of t while training (*= scale)')
flags.DEFINE_float('t_learn_start', 5, 'The time when to begin training (*= scale)')
flags.DEFINE_float('learning_rate_decay_step', 5, 'The learning rate of training (*= scale)')

# Optimizer
flags.DEFINE_float('learning_rate', 0.00025, 'The learning rate of training')
flags.DEFINE_float('learning_rate_minimum', 0.00025, 'The minimum learning rate of training')
flags.DEFINE_float('learning_rate_decay', 0.96, 'The decay of learning rate of training')
flags.DEFINE_float('decay', 0.99, 'Decay of RMSProp optimizer')
flags.DEFINE_float('momentum', 0.0, 'Momentum of RMSProp optimizer')
flags.DEFINE_float('gamma', 0.99, 'Discount factor of return')
flags.DEFINE_float('beta', 0.01, 'Beta of RMSProp optimizer')

# Debug
flags.DEFINE_boolean('display', True, 'Whether to do display the game screen or not')
flags.DEFINE_string('log_level', 'INFO', 'Log level [DEBUG, INFO, WARNING, ERROR, CRITICAL]')
flags.DEFINE_integer('random_seed', 123, 'Value of random seed')
flags.DEFINE_string('tag', '', 'The name of tag for a model, only for debugging')
flags.DEFINE_string('gpu_fraction', '2/3', 'idx / # of gpu fraction e.g. 1/3, 2/3, 3/3')


def calc_gpu_fraction(fraction_string):
  idx, num = fraction_string.split('/')
  idx, num = float(idx), float(num)

  fraction = 1 / (num - idx + 1)
  print (" [*] GPU : %.4f" % fraction)
  return fraction

conf = flags.FLAGS

if conf.agent_type == 'DQN':
  from agents.deep_q import DeepQ
  TrainAgent = DeepQ
else:
  raise ValueError('Unknown agent_type: %s' % conf.agent_type)

logger = logging.getLogger()
logger.propagate = False
logger.setLevel(conf.log_level)

# set random seed
tf.set_random_seed(conf.random_seed)
random.seed(conf.random_seed)

# Environment 
class StageEnvironment(object):
  def __init__(self, max_random_start,
               observation_dims, data_format, display):
    
    self.max_random_start = max_random_start
    self.action_size = 28

    self.display = display
    self.data_format = data_format
    self.observation_dims = observation_dims

    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1],1), np.uint8)
    
    #####################
    # Followings are not used yet
    self.syncLaser = False
    self.syncPose = False
    self.syncGoal = False
    self.syncDir = False

    self.sentTime = 0
    #####################

    self.poseX = 0.0
    self.poseY = 0.0
    self.prevPoseX = 0.0
    self.prevPoseY = 0.0
    self.goalX = 0.0
    self.goalY = 0.0
    self.dir = 0.0
    self.prevDist = 0.0
    self.terminal = 0

    rospy.wait_for_service('reset_positions')
    self.resetStage = rospy.ServiceProxy('reset_positions', EmptySrv)

    # Publishers:
    self.pub_action_ = rospy.Publisher("dqn/selected_action",Int8,queue_size=1)
    self.pub_new_goal_ = rospy.Publisher("dqn/new_goal",EmptyMsg,queue_size=1)
    
    # Subscribers:
    rospy.Subscriber('bridge/laser_image', Image, self.laserCB,queue_size=1)
    rospy.Subscriber('bridge/current_dir', Float32, self.directionCB,queue_size=1)
    rospy.Subscriber('bridge/impact', EmptyMsg, self.impactCB,queue_size=1)
    rospy.Subscriber('bridge/current_pose', Pose, self.positionCB,queue_size=1)
    rospy.Subscriber('bridge/goal_pose', Pose, self.targetCB,queue_size=1)

  
  
  def new_game(self):
    rospy.wait_for_service('reset_positions')
    self.resetStage()
    self.terminal = 0
    #newStateMSG = EmptyMsg()
    #self.pub_new_goal_.publish( newStateMSG)
    cv2.waitKey(30)
    return self.preprocess(), 0, False

  def new_random_game(self):
    # TODO: maybe start from a random position not just reset the robot's position to initial point
    # which needs some edit in stage_ros
    return self.new_game()

  def step(self, action, is_training=False):
    self.prevPoseX = self.poseX
    self.prevPoseY = self.poseY

    if action == -1:
      # Step with random action
      action = int(random.random()*(self.action_size))

    msg = Int8()
    msg.data = action
    self.pub_action_.publish( msg)

    if self.display:
      cv2.imshow("Screen", self.screen)
    cv2.waitKey(9)

    dist = (self.poseX - self.goalX)**2 + (self.poseY - self.goalY)**2
    reward = (self.prevDist - dist)/10.0
    self.prevDist = dist

    if self.terminal == 1:
      reward -= 900
      #self.new_random_game()

    if dist < 0.9:
      reward += 90
      newStateMSG = EmptyMsg()
      self.pub_new_goal_.publish( newStateMSG)
      # cv2.waitKey(30)

    # Add whatever info you want
    info = ""

    #rospy.loginfo("Episede ended, reward: %g", reward)
    return self.screen, reward, self.terminal, info
    #observation, reward, terminal, info = self.env.step(action)
    #return self.preprocess(observation), reward, terminal, info

  def preprocess(self):
    return self.screen

  def laserCB(self, data):
    try:
      self.screen = np.squeeze(bridge.imgmsg_to_cv2(data, "mono8"))
    except CvBridgeError as e:
      print(e)

  def directionCB(self, data):
    self.dir = data

  def positionCB( self, data):
    self.poseX = data.position.x
    self.poseY = data.position.y

  def targetCB( self, data):
    self.goalX = data.position.x
    self.goalY = data.position.y

  def impactCB( self, data):
    self.terminal = 1


def main(_):
  # preprocess
  conf.observation_dims = eval(conf.observation_dims)

  for flag in ['memory_size', 't_target_q_update_freq', 't_test',
               't_ep_end', 't_train_max', 't_learn_start', 'learning_rate_decay_step']:
    setattr(conf, flag, getattr(conf, flag) * conf.scale)

  if conf.use_gpu:
    conf.data_format = 'NCHW'
  else:
    conf.data_format = 'NHWC'

  model_dir = get_model_dir(conf,
      ['use_gpu', 'max_random_start', 'n_worker', 'is_train', 'memory_size', 'gpu_fraction',
       't_save', 't_train', 'display', 'log_level', 'random_seed', 'tag', 'scale'])

  # start
  gpu_options = tf.GPUOptions(
      per_process_gpu_memory_fraction=calc_gpu_fraction(conf.gpu_fraction))

  with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
    env = StageEnvironment(conf.max_random_start, conf.observation_dims, conf.data_format, conf.display)

    if conf.network_header_type in ['nature', 'nips']:
      pred_network = CNN(sess=sess,
                         data_format=conf.data_format,
                         history_length=conf.history_length,
                         observation_dims=conf.observation_dims,
                         output_size=env.action_size,
                         network_header_type=conf.network_header_type,
                         name='pred_network', trainable=True)
      target_network = CNN(sess=sess,
                           data_format=conf.data_format,
                           history_length=conf.history_length,
                           observation_dims=conf.observation_dims,
                           output_size=env.action_size,
                           network_header_type=conf.network_header_type,
                           name='target_network', trainable=False)
    elif conf.network_header_type == 'mlp':
      pred_network = MLPSmall(sess=sess,
                              observation_dims=conf.observation_dims,
                              history_length=conf.history_length,
                              output_size=env.action_size,
                              hidden_activation_fn=tf.sigmoid,
                              network_output_type=conf.network_output_type,
                              name='pred_network', trainable=True)
      target_network = MLPSmall(sess=sess,
                                observation_dims=conf.observation_dims,
                                history_length=conf.history_length,
                                output_size=env.action_size,
                                hidden_activation_fn=tf.sigmoid,
                                network_output_type=conf.network_output_type,
                                name='target_network', trainable=False)
    else:
      raise ValueError('Unkown network_header_type: %s' % (conf.network_header_type))

    stat = Statistic(sess, conf.t_test, conf.t_learn_start, model_dir, pred_network.var.values())
    agent = TrainAgent(sess, pred_network, env, stat, conf, target_network=target_network)

    if conf.is_train:
      agent.train(conf.t_train_max)
    else:
      agent.play(conf.ep_end)


if __name__ == '__main__':
  # Initialize the node with rospy
  rospy.init_node('dqn_node')

  tf.app.run()
  # spin to keep the script for exiting
  rospy.spin()
