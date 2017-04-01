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

#pose_list = [[-13, 20], [-13,17], [-13,14], [-13,10], [-13,1], [-13,-10], [-13,-24], [-12,-3], [-12,-19], [-12,-19], [-8,-4], [-8,-19], [-8,-26], [-3,6], [-3,13], [-3,21], [-3,-5], [-3,-26], [-62,-14], [-62,-17], [-62,-22], [-56,-10], [-56,-18], [-56,-23], [-53,-10], [-53,-19], [-50,-10], [-50,-23], [-47    ,-10], [-45,-18], [-45,-23], [-42,-9], [-38,-4], [-38,-18], [-38,-22], [-38,-26], [-30,-4], [-30,-10], [-30,-18], [-30,-22], [-30,-26], [-26,-18], [-23,-23], [-20,-10], [-20,-18], [-18,-11], [-18,-23], [-17,-18], [0,14], [0,0], [0,-14], [0,-22], [3,6], [3,-1], [3,-6], [3,-18], [8,-19], [10,-24], [14,-9], [    14,-19], [17,-23], [17,-18], [17,-11], [20,-5], [20,-8], [20,-11], [20,-22], [20,-28], [25,-5], [25,-11], [25,-28], [28,-5], [35,-11], [35,-19], [35,-23], [39,-10], [39,-20], [39,-24], [45,-10], [45,-20], [45,-24], [50,-11], [50,-17], [52,-14], [52,-17], [52,-20], [52,-24], [46,-14], [38,-14], [34,-14],  [    20,-14], [12,-14], [6,-14], [0,-14], [-9,-14], [-18,-14], [-27,-14], [-36,-14], [-48,-14], [-60,-14],]
pose_list = [[-17,6], [-17,1], [-17,-4], [-16,-9], [-13,3], [-13,-2], [-13,-6], [-13,-11], [-11,14], [-11,10], [-11,7], [-11,4], [-11,2], [-11,-2], [-11,-8], [-11,-11], [-11,-14], [-8,15], [-8,10], [-8,4], [-8,1], [-8,-3], [-8,-6], [-8,-11], [-8,-14], [-7,0], [-4,18], [-4,14], [-4,10], [-4,7], [-4,6], [-4,1], [-4,-1], [-4,-3], [-4,-5], [-4,-7], [-4,-18], [0,18], [0,16], [0,12], [0,9], [0,6], [0,-1], [0,-2], [0,-3], [0,-5], [0,-8], [0,-12], [0,-16], [0,-18], [2,18], [2,14], [2,10], [2,0], [2,-2], [2,-5], [2,-8], [2,-18], [5,16], [5,11], [5,1], [5,-5], [5,-10], [5,-13], [5,-17], [9,15], [9,10], [9,2], [9,-2],[9,-5], [9,-7], [9,-10], [9,-13], [12,14], [12,2], [12,-2], [12,-10], [12,-13], [14,11], [14,7], [14,3], [14,-3], [14,-7], [14,-10], [17,8], [17,3], [17,-2], [17,-6]]
class StageEnvPlay(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, observation_dims=[66, 66]):
    self.viewer = None
    self.observation_dims = observation_dims
    # action space: continious forward speed [0, 1.8] and turning speed [-1, 1] 
    self.action_space = spaces.Tuple(( spaces.Box( low=0, high=2.0, shape=1), spaces.Box( low=-1.0, high=1.0, shape=1))) 
    # self.action_space = spaces.MultiDiscrete( [ [0,3], [0,6]])
    # self.action_space = spaces.Discrete( 28)
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
    self.I = np.zeros(( self.observation_dims[0], self.observation_dims[1]))
    self.J = np.zeros(( self.observation_dims[0], self.observation_dims[1]))
    for i in range( 0, self.observation_dims[0]):
      for j in range( 0, self.observation_dims[1]):
        self.I[i][j] = (i- self.observation_dims[0]/2)
        self.J[i][j] = (j - self.observation_dims[1]/2)




    #For babyStep training
    self.r = 1
    self.ang = 0 #np.pi*2

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
      self.actionToVel( action[0], action[1])
      # self.actionToVelDisc( action)
      self.readyForNewData = True

    dist = np.sqrt( (self.robotPose.position.x - self.goalPose.position.x)**2 + (self.robotPose.position.y - self.goalPose.position.y)**2)
    #near obstacle penalty factor:
    #nearObstPenalty = self.minFrontDist - 1.5
    #reward = max( 0, ((self.prevDist - dist + 1.8)*3/dist)+min( 0, nearObstPenalty))
    #self.prevDist = dist
    reward = 0
    if dist < 0.3:
      reward += 1
      #self.chooseNewTarget()
      #rwd = Float64()
      #rwd.data = 10101.963
      #self.pub_rew_.publish( rwd)
      self.selectNewGoal()
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
      #rewd = Float64()
      #rewd.data = self.ep_reward
      #self.pub_rew_.publish( rewd)
      self.sendTerminal = True
    
    wait_from = rospy.get_rostime()
    while( self.readyForNewData == True):
      if rospy.get_rostime().secs - wait_from.secs > 1:
        wait_from = rospy.get_rostime()
        self.actionToVelDisc( action)
      pass

    return self.screen, reward, self.sendTerminal, info

  def _reset(self):
    self.resetStage()
    self.terminal = False
    self.sendTerminal = False
    self.selectNewGoal()
    #self.chooseNewTarget()
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
    theta = self.ang*random.random()+(float(self.robot_id)*np.pi/4.0)
    dist = random.random()*self.r
    self.goalPose.position.x = dist*np.cos( theta)
    self.goalPose.position.y = dist*np.sin( theta)
    self.pub_goal_.publish( self.goalPose)

  def chooseNewTarget( self):
    choice = random.choice( pose_list)
    self.goalPose.position.x = choice[0]
    self.goalPose.position.y = choice[1]
    self.pub_goal_.publish( self.goalPose)

  def actionToVel( self, forward, turn):
    msg = Twist()
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = turn
    msg.linear.x = forward+1
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
    c = 5.125
    rad = int(data.laser.range_max*c)
    for i in range(0, 360):
      for j in range( 0, rad):
        if data.laser.ranges[i]*c >= j:
          self.setPixel( x + int(j* np.cos( np.pi*i/180.0)), y + int( j* np.sin( np.pi*i/180.0)) ,192)
      if data.laser.ranges[i] < data.laser.range_max:
        self.setPixel( x + int( c*data.laser.ranges[i]*np.cos( np.pi*i/180.0)), y + int( c*data.laser.ranges[i]*np.sin( np.pi*i/180.0)), 0)
    
    # Gradient:
#    R = np.matrix('{} {}; {} {}'.format(np.cos( np.radians( -dirToTarget)), -np.sin( np.radians( -dirToTarget)), np.sin( np.radians( -dirToTarget)), np.cos( np.radians( -dirToTarget))))
    tempScr = np.zeros(( self.observation_dims[0], self.observation_dims[1]))
    sin = np.sin( np.pi+yaw)
    cos = np.cos( np.pi+yaw)
    #for i in range( 0, self.observation_dims[0]):
    #  for j in range( 0, self.observation_dims[1]):
    #    x =  (i- self.observation_dims[0]/2) * cos + (j - self.observation_dims[1]/2) * -sin
    #    y =  (i- self.observation_dims[0]/2) * sin + (j - self.observation_dims[1]/2) * cos
    #    tempScr[ i][ j] = np.sqrt( (self.robotPose.position.x + x/c - self.goalPose.position.x)**2 + (self.robotPose.position.y +  y /c - self.goalPose.position.y)**2)
    sx = self.robotPose.position.x + (self.I*cos - self.J*sin)/c - self.goalPose.position.x
    sy = self.robotPose.position.y + (self.I*sin + self.J*cos)/c - self.goalPose.position.y
    tempScr = np.sqrt( sx**2 + sy**2)    
    tempScr -= np.min( tempScr)
    tempScr /= np.max( tempScr)
    self.screen += (63.0*(np.ones_like(tempScr)-tempScr)).astype(np.uint8)
    #for i in range( 0, self.observation_dims[0]):
    #  for j in range( 0, self.observation_dims[1]):
    #    self.addPixel( i, j, 63.0*(1-tempScr[ i][ j])) 
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
