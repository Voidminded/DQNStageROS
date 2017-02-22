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

#The rest of stuff
import numpy as np

class StageEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, robot_name, observation_dims=[80, 80]):
  	self.observation_dims = observation_dims
  	# action space: continious forward speed [0, 1.8] and turning speed [-1, 1] 
    self.action_space = spaces.Tuple(( spaces.Box( low=0, high=1.8, shape=1), spaces.Box( low=-1, high=1, shape=1))) 
    self.observation_space = spaces.Box(low=0, high=255, shape=( observation_dims[0], observation_dims[1], 1))
    #self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1]), np.uint8)
    self.robotPose = Pose()
    self.goalPose = Pose()
    self.terminal = False
    self.readyForNewData = True
    self.minFrontDist = 6
    self.numWins = 0


    #For babyStep training
    self.r = 1
    self.ang = 0

    rospy.wait_for_service( robot_name +'/reset_positions')
    self.resetStage = rospy.ServiceProxy( robot_name +'/reset_positions', EmptySrv)

    # Subscribers:
    rospy.Subscriber( robot_name+'/input_data', stage_message, self.stageCB, queue_size=10)

    # publishers:
    self.pub_vel_ = rospy.Publisher( robot_name+'/cmd_vel', Twist, queue_size = 1)
    self.pub_rew_ = rospy.Publisher( robot_name+'/lastreward',Float64,queue_size=10)
    self.pub_goal_ = rospy.Publisher( robot_name+'/target', Pose, queue_size = 15) 

  def _step(self, action):
    ...
  def _reset(self):
    self.resetStage()
    self.terminal = 0
    self.selectNewGoal()
    self.readyForNewData = True
    self.numWins = 0
    self.actionToVel( 0.0, 0.0)
    return self.screen, 0, 0
    
  def _render(self, mode='human', close=False):
    ...

  def selectNewGoal( self):
  	theta =  self.ang*random.random()
    self.goalPose.position.x = random.random()*self.r*np.cos( theta)
    self.goalPose.position.y = random.random()*self.r*np.sin( theta)
    self.pub_goal_.publish( self.goalPose)

  def actionToVel( self, forward, turn):
  	msg = Twist()
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = turn
    msg.linear.x = forward
    msg.linear.y = 0
    msg.linear.z = 0
    self.pub_vel_.publish( msg)
