#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty as EmptySrv
from tf import transformations
import message_filters

import cv2
import random
import logging
import numpy as np
import copy

import tensorflow as tensor
from utils import get_model_dir
from networks.cnn import CNN
from networks.mlp import MLPSmall
from agents.statistic import Statistic

logger = logging.getLogger(__name__)

flags = tensor.app.flags

Map_Max_Dist = 40.0

# Deep q Network
flags.DEFINE_boolean('use_gpu', True, 'Whether to use gpu or not. gpu use NHWC and gpu use NCHW for data_format')
flags.DEFINE_string('agent_type', 'DQN', 'The type of agent [DQN]')
flags.DEFINE_boolean('double_q', False, 'Whether to use double Q-learning')
flags.DEFINE_string('network_header_type', 'nature', 'The type of network header [mlp, nature, nips]')
flags.DEFINE_string('network_output_type', 'normal', 'The type of network output [normal, dueling]')

# Environment
flags.DEFINE_string('env_name', 'Stage', 'The name of gym environment to use')
flags.DEFINE_integer('max_random_start', 30, 'The maximum number of NOOP actions at the beginning of an episode')
flags.DEFINE_integer('history_length', 4, 'The length of history of observation to use as an input to DQN')
flags.DEFINE_integer('max_r', +999999999, 'The maximum value of clipped reward')
flags.DEFINE_integer('min_r', -999999999, 'The minimum value of clipped reward')
flags.DEFINE_string('observation_dims', '[80, 80]', 'The dimension of gym observation')
flags.DEFINE_boolean('random_start', False, 'Whether to start with random state')

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
flags.DEFINE_integer('t_train_freq', 1, '')

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
flags.DEFINE_boolean('display', False, 'Whether to do display the game screen or not')
flags.DEFINE_string('log_level', 'DEBUG', 'Log level [DEBUG, INFO, WARNING, ERROR, CRITICAL]')
flags.DEFINE_integer('random_seed', 123, 'Value of random seed')
flags.DEFINE_string('tag', '', 'The name of tag for a model, only for debugging')
flags.DEFINE_string('gpu_fraction', '3/3', 'idx / # of gpu fraction e.g. 1/3, 2/3, 3/3')


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
tensor.set_random_seed(conf.random_seed)
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

    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1]), np.uint8)
    
    #self.dirToTarget = 0
    self.robotPose = Pose()
    self.goalPose = Pose()
    self.robotRot = 0.0
    self.prevDist = 0.0
    self.boom = False
    self.numWins = 0

    self.terminal = 0
    self.sendTerminal = 0
    
    self.readyForNewData = True

    rospy.wait_for_service('reset_positions')
    self.resetStage = rospy.ServiceProxy('reset_positions', EmptySrv)

    
    # Subscribers:
   # rospy.Subscriber('base_scan', LaserScan, self.scanCB,queue_size=1)
   # rospy.Subscriber('base_pose_ground_truth', Odometry, self.poseUpdateCB, queue_size=1)
    
    # trying time sync:
    sub_scan_ = message_filters.Subscriber('base_scan', LaserScan)
    sub_pose_ = message_filters.Subscriber('base_pose_ground_truth', Odometry)
    ts = message_filters.TimeSynchronizer( [sub_scan_, sub_pose_], 1)
    ts.registerCallback( self.syncedCB)

    # publishers:
    self.pub_vel_ = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    self.pub_rew_ = rospy.Publisher("lastreward",Float64,queue_size=1) 


  def new_game(self):
    self.resetStage()
    rospy.sleep(0.3)
    self.terminal = 0
    self.sendTerminal = 0
    #Select a new goal
    theta = 2.0 * np.pi * random.random()
    r = random.random()*21
    self.goalPose.position.x = r*np.cos( theta)
    self.goalPose.position.y = r*np.sin( theta)
    self.readyForNewData = False
    self.numWins = 0

    self.prevDist = (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2
    return self.screen, 0, False

  def new_random_game(self):
    # TODO: maybe start from a random position not just reset the robot's position to initial point
    # which needs some edit in stage_ros
    return self.new_game()
    #pass

  def step(self, action, is_training):
    if action == -1:
      # Step with random action
      action = int(random.random()*(self.action_size))

    self.actionToVel( action)
    self.readyForNewData = True

    if self.display:
      cv2.imshow("Screen", self.screen)
      #cv2.waitKey(30)

    dist = (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2
    reward = self.prevDist - dist
    self.prevDist = dist

    if self.terminal == 1:
      reward -= 900
      self.boom = True
      rewd = Float64()
      rewd.data = copy.deepcopy( reward)
      self.pub_rew_.publish( rewd)

      self.sendTerminal = 1

    if dist < 0.9:
      reward += 300
      #Select a new goal
      theta = 2.0 * np.pi * random.random()
      r = random.random()*21
      self.goalPose.position.x = r*np.cos( theta)
      self.goalPose.position.y = r*np.sin( theta)
      self.prevDist = (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2
      rwd = Float64()
      rwd.data = 10101.963
      self.pub_rew_.publish( rwd)
      if self.numWins == 99:
        reward += 9000
        rewd = Float64()
        rewd.data = copy.deepcopy( reward)
        self.pub_rew_.publish( rewd)
        self.sendTerminal = 1
    
    # Add whatever info you want
    info = ""

    #rospy.loginfo("Episede ended, reward: %g", reward)
   # while(True): #self.clock == self.lastClock): ----> aghimed !!!
   #   pass
    
    #if self.terminal == 2:
      #self.sendTerminal = 1

    #if self.terminal == 1:
      #self.terminal = 2
   

    while( self.readyForNewData == True):
      pass
    
    return self.screen, reward, self.sendTerminal, info
    
    #observation, reward, terminal, info = self.env.step(action)
    #return self.preprocess(observation), reward, terminal, info

  def preprocess(self):
    return self.screen

#  def scanCB( self, scan):
#    if len(scan.ranges) < 1:
#      return
#    #self.screen = np.zeros(( self.observation_dims[0], self.observation_dims[1], 1), np.uint8)
#    self.screen[:] = 128
#    x = int(scan.range_max* 10)
#    y = int(scan.range_max*10)
#    rad = int(scan.range_max*10)
#    boomed = False
#    
#    # Occupancy grid:
#    for i in range(0, 360):
#      for j in range( 0, rad):
#        if scan.ranges[i]*10 >= j:
#          self.screen[ x + int(j* np.cos( np.pi*i/180.0)), y + int( j* np.sin( np.pi*i/180.0))] = 255
#      if scan.ranges[i]*10 < scan.range_max:
#        self.screen[ x + int( 10*scan.ranges[i]*np.cos( np.pi*i/180)), y + int( 10*scan.ranges[i]*np.sin( np.pi*i/180))] = 0
#      if scan.ranges[i] < 0.66:
#        boomed = True
#   
#    #steering guide:
#    for i in range( rad*2, self.observation_dims[0]):
#      stir = (self.dirToTarget*40/180)+40
#      minStir = min( stir, 40)
#      maxStir = max( stir, 40)
#      for j in range( 0, 80):
#        if j > minStir and j < maxStir:
#          self.screen[i ,j] = 0
#        else:
#          self.screen[ i, j] = 255
#    
#    #Distance info:
#    dist = np.sqrt( (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2)
#    normDist = 72*dist/Map_Max_Dist
#    for j in range( rad*2, self.observation_dims[1]):
#      for i in range(0, 72):
#        if i <= normDist:
#          self.screen[i, j] = 0
#        else:
#          self.screen[i, j] = 255 
#
#    if boomed == True:
#      self.terminal = 1 
#
#  def poseUpdateCB(self, odom):
#    self.robotPose = odom.pose.pose
#    quatOri = (self.robotPose.orientation.x, self.robotPose.orientation.y, self.robotPose.orientation.z, self.robotPose.orientation.w)
#    ( _, _, yaw) = transformations.euler_from_quaternion( quatOri)
#    robotOri =  180*yaw/np.pi
#    angleBetween = 180* np.arctan2( self.goalPose.position.y - self.robotPose.position.y, self.goalPose.position.x - self.robotPose.position.x) /np.pi
#    self.dirToTarget = self.constrainAngle( robotOri- angleBetween)
#    
  def constrainAngle( self, x):
    x = np.fmod( x+180, 360)
    if x < 0:
      x+= 360
    return x-180

  def actionToVel( self, action):
    if action < 0 or action >= self.action_size:
      rospy.logerr( "Invalid action %d", action)
    else:
      w = ((action % 7)-3)/3.0
      f = 0.3*(action % 7)
      msg = Twist()
      msg.angular.x = 0
      msg.angular.y = 0
      msg.angular.z = w
      msg.linear.x = f
      msg.linear.y = 0
      msg.linear.z = 0
      self.pub_vel_.publish( msg)

  def syncedCB( self, scan, odom):
    if len(scan.ranges) < 1 or self.readyForNewData == False:
      #print "limitted by GPU !!!!"
      return
    
    # pose stuff
    self.robotPose = odom.pose.pose
    quatOri = (self.robotPose.orientation.x, self.robotPose.orientation.y, self.robotPose.orientation.z, self.robotPose.orientation.w)
    ( _, _, yaw) = transformations.euler_from_quaternion( quatOri)
    robotOri =  180*yaw/np.pi
    angleBetween = 180* np.arctan2( self.goalPose.position.y - self.robotPose.position.y, self.goalPose.position.x - self.robotPose.position.x) /np.pi
    dirToTarget = self.constrainAngle( robotOri- angleBetween)

    self.screen[:] = 128
    x = int(scan.range_max* 10)
    y = int(scan.range_max*10)
    rad = int(scan.range_max*10)
    boomed = False
    
    # Occupancy grid:
    for i in range(0, 360):
      for j in range( 0, rad):
        if scan.ranges[i]*10 >= j:
          self.screen[ x + int(j* np.cos( np.pi*i/180.0)), y + int( j* np.sin( np.pi*i/180.0))] = 255
      if scan.ranges[i]*10 < scan.range_max:
        self.screen[ x + int( 10*scan.ranges[i]*np.cos( np.pi*i/180)), y + int( 10*scan.ranges[i]*np.sin( np.pi*i/180))] = 0
      if scan.ranges[i] < 0.66:
        boomed = True
   
    #steering guide:
    for i in range( rad*2, self.observation_dims[0]):
      stir = (dirToTarget*40/180)+40
      minStir = min( stir, 40)
      maxStir = max( stir, 40)
      for j in range( 0, 80):
        if j > minStir and j < maxStir:
          self.screen[i ,j] = 0
        else:
          self.screen[ i, j] = 255
    
    #Distance info:
    dist = np.sqrt( (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2)
    normDist = 72*dist/Map_Max_Dist
    for j in range( rad*2, self.observation_dims[1]):
      for i in range(0, 72):
        if i <= normDist:
          self.screen[i, j] = 0
        else:
          self.screen[i, j] = 255 

    if boomed == True:
      if self.boom == False:
        self.terminal = 1 
    else:
      self.boom = False
    self.readyForNewData = False

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
  gpu_options = tensor.GPUOptions(
      per_process_gpu_memory_fraction=calc_gpu_fraction(conf.gpu_fraction))

  with tensor.Session(config=tensor.ConfigProto(gpu_options=gpu_options)) as sess:
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
                              hidden_activation_fn=tensor.sigmoid,
                              network_output_type=conf.network_output_type,
                              name='pred_network', trainable=True)
      target_network = MLPSmall(sess=sess,
                                observation_dims=conf.observation_dims,
                                history_length=conf.history_length,
                                output_size=env.action_size,
                                hidden_activation_fn=tensor.sigmoid,
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

  tensor.app.run()
