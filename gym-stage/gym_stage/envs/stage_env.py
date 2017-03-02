#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#ROS stuff
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty as EmptySrv
from dqn_stage_ros.msg import stage_message
from tf import transformations

#The rest of stuff
import numpy as np
import random

class StageEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, observation_dims=[42, 42]):
    self.viewer = None
    self.observation_dims = observation_dims
    # action space: continious forward speed [0, 1.8] and turning speed [-1, 1] 
    # self.action_space = spaces.Tuple(( spaces.Box( low=0, high=1.8, shape=1), spaces.Box( low=-1, high=1, shape=1))) 
    # self.action_space = spaces.MultiDiscrete( [ [0,3], [0,6]])
    self.action_space = spaces.Discrete( 28)
    self.observation_space = spaces.Box(low=0, high=255, shape=( observation_dims[0], observation_dims[1], 1))
    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1]), np.uint8)
    self.robotPose = Pose()
    self.goalPose = Pose()
    self.terminal = False
    self.sendTerminal = False
    self.readyForNewData = True
    self.minFrontDist = 6
    self.numWins = 0
    self.ep_reward = 0


    #For babyStep training
    self.r = 1
    self.ang = 0 

  def initRos( self, robot_id):
    self.robot_id = robot_id
    robot_name = 'r'+robot_id
    rospy.init_node( robot_name+'_node', disable_signals=True)
    rospy.wait_for_service( robot_name +'/reset_positions')
    self.resetStage = rospy.ServiceProxy( robot_name +'/reset_positions', EmptySrv)

    # Subscribers:
    rospy.Subscriber( robot_name+'/input_data', stage_message, self.stageCB, queue_size=10)

    # publishers:
    self.pub_vel_ = rospy.Publisher( robot_name+'/cmd_vel', Twist, queue_size = 1)
    self.pub_rew_ = rospy.Publisher( robot_name+'/lastreward',Float64,queue_size=10)
    self.pub_goal_ = rospy.Publisher( 't'+robot_id+'/target', Pose, queue_size = 15)

  def _step(self, action):
    if self.terminal == False:
      # self.actionToVel( action[0], action[1])
      self.actionToVelDisc( action)
      self.readyForNewData = True

    dist = np.sqrt( (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2)
    #near obstacle penalty factor:
    #nearObstPenalty = self.minFrontDist - 1.5
    #reward = max( 0, ((self.prevDist - dist + 1.8)*3/dist)+min( 0, nearObstPenalty))
    #self.prevDist = dist
    reward = 0
    if dist < 0.3:
      reward += 1
      self.selectNewGoal()
      rwd = Float64()
      rwd.data = 10101.963
      self.pub_rew_.publish( rwd)
      self.numWins += 1
      if self.numWins >= 99:
        if self.r < 18:
          self.r += 1
        elif self.ang < 2*np.pi:
          self.ang += 0.1
        self.nimWins = 0
      self.resetStage()
    
    # Add whatever info you want
    info = {}
    self.ep_reward += reward
    if self.terminal == True:
      reward = -1 
      rewd = Float64()
      rewd.data = self.ep_reward
      self.pub_rew_.publish( rewd)
      self.sendTerminal = True

    while( self.readyForNewData == True):
      pass

    return self.screen, reward, self.sendTerminal, info

  def _reset(self):
    self.resetStage()
    self.terminal = False
    self.sendTerminal = False
    self.selectNewGoal()
    self.readyForNewData = False
    self.numWins = 0
    self.actionToVel( 0.0, 0.0)
    self.ep_reward = 0
    return self.screen, 0, 0

  def _render(self, mode='human', close=False):
    if close:
      if self.viewer is not None:
        self.viewer.close()
        self.viewer = None
    else:
      from gym.envs.classic_control import rendering
      if self.viewer is None:
        self.viewer = rendering.SimpleImageViewer()
      s = np.concatenate( (self.screen[:,:,None],self.screen[:,:,None],self.screen[:,:,None]), axis = 2)
      self.viewer.imshow( s)

  def _close( self):
    rospy.signal_shutdown("Done")

  def selectNewGoal( self):
    theta =  self.ang*random.random()+(float(self.robot_id)*np.pi/4.0)
    dist = random.random()*self.r
    self.goalPose.position.x = dist*np.cos( theta)
    self.goalPose.position.y = dist*np.sin( theta)
    self.pub_goal_.publish( self.goalPose)

  def actionToVel( self, forward, turn):
    msg = Twist()
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = (float(turn) - 3.0) / 3
    msg.linear.x = forward * 0.3
    msg.linear.y = 0
    msg.linear.z = 0
    self.pub_vel_.publish( msg)

  def actionToVelDisc( self, action):
    if action < 0 or action >= self.action_space.n:
      rospy.logerr( "Invalid action %d", action)
    else:
      w = ((action % 7)-3)/3.0
      f = 0.3*(action / 7)
      msg = Twist()
      msg.angular.x = 0
      msg.angular.y = 0
      msg.angular.z = w
      msg.linear.x = f
      msg.linear.y = 0
      msg.linear.z = 0
      self.pub_vel_.publish( msg)

  def stageCB( self, data):
    # pose stuff
    self.robotPose = data.position.pose
    quatOri = (self.robotPose.orientation.x, self.robotPose.orientation.y, self.robotPose.orientation.z, self.robotPose.orientation.w)
    ( _, _, yaw) = transformations.euler_from_quaternion( quatOri)
    robotOri =  180*yaw/np.pi
    angleBetween = 180* np.arctan2( self.goalPose.position.y - self.robotPose.position.y, self.goalPose.position.x - self.robotPose.position.x) /np.pi
    dirToTarget = self.constrainAngle( robotOri- angleBetween)

    self.screen[:] = 128
    x = self.observation_dims[0]/2
    y = self.observation_dims[1]/2
    
    # Occupancy grid:
    c = 5.25
    rad = int(data.laser.range_max*c)
    for i in range(0, 360):
      for j in range( 0, rad):
        if data.laser.ranges[i]*c >= j:
          self.setPixel( x + int(j* np.cos( np.pi*i/180.0)), y + int( j* np.sin( np.pi*i/180.0)) ,192)
      if data.laser.ranges[i] < data.laser.range_max:
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180)), y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180)), 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180)), y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-1, 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180)), y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-2, 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-1, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180)), 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-1, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-1, 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-1, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-2, 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-2, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180)), 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-2, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-1, 0)
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180))-2, y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180))-2, 0)
    
    # Gradient:
#    R = np.matrix('{} {}; {} {}'.format(np.cos( np.radians( -dirToTarget)), -np.sin( np.radians( -dirToTarget)), np.sin( np.radians( -dirToTarget)), np.cos( np.radians( -dirToTarget))))
    tempScr = np.zeros(( self.observation_dims[0], self.observation_dims[1]))
    for i in range( 0, self.observation_dims[0]):
      for j in range( 0, self.observation_dims[1]):
        x = int( (i- self.observation_dims[0]/2) * np.cos( np.pi+yaw) + (j - self.observation_dims[1]/2) * -np.sin( np.pi+yaw))
        y = int( (i- self.observation_dims[0]/2) * np.sin( np.pi+yaw) + (j - self.observation_dims[1]/2) * np.cos( np.pi+yaw))
        tempScr[ i][ j] = np.sqrt( (self.robotPose.position.x + x/10.0 - self.goalPose.position.x)**2 + (self.robotPose.position.y +  y /10.0 - self.goalPose.position.y)**2)
    tempScr -= np.min( tempScr)
    tempScr /= np.max( tempScr)
    for i in range( 0, self.observation_dims[0]):
      for j in range( 0, self.observation_dims[1]):
        self.addPixel( i, j, 63.0*(1-tempScr[ i][ j])) 
    if data.collision == True:
      self.terminal = 1 
    self.minFrontDist = data.minFrontDist
    self.readyForNewData = False

  def setPixel( self, x, y, val):
    if x < 0 or x >= self.observation_dims[0] or y < 0 or y >= self.observation_dims[1]:
      return
    self.screen[ x, y] = val
    
  def addPixel( self, x, y, val):
    if x < 0 or x >= self.observation_dims[0] or y < 0 or y >= self.observation_dims[1]:
      return
    self.screen[ x, y] += val

  def constrainAngle( self, x):
    x = np.fmod( x+180, 360)
    if x < 0:
      x+= 360
    return x-180
