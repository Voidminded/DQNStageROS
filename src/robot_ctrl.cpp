#include <stage.hh>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dqn_stage_ros/stage_message.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

using namespace Stg;

struct ModelRobot
{
  ModelPosition* pos;
  ModelRanger* laser;
  Pose resetPose;
};

ModelRobot* robot;
usec_t stgSpeedTime;

ros::NodeHandle* n;
ros::Publisher pub_state_;
ros::Subscriber sub_vel_;
ros::ServiceServer reset_srv_;

geometry_msgs::PoseStamped rosCurPose;
sensor_msgs::LaserScan rosLaserData;
bool collision = false;
bool allowNewMsg = true;
double minFrontDist;
ros::Time lastSentTime;

void stgPoseUpdateCB( Model* mod, ModelRobot* robot)
{
  geometry_msgs::PoseStamped positionMsg;
  positionMsg.pose.position.x = robot->pos->GetPose().x;
  positionMsg.pose.position.y = robot->pos->GetPose().y;
  positionMsg.pose.position.z = robot->pos->GetPose().z;
  positionMsg.pose.orientation = tf::createQuaternionMsgFromYaw( robot->pos->GetPose().a);
  positionMsg.header.stamp = ros::Time::now();
  rosCurPose = positionMsg;
}

void stgLaserCB( Model* mod, ModelRobot* robot)
{
  sensor_msgs::LaserScan laserMsgs;
  const Stg::ModelRanger::Sensor& sensor = robot->laser->GetSensors()[0];
  if( sensor.ranges.size() )
    {
      // Translate into ROS message format and publish
      laserMsgs.angle_min = -sensor.fov/2.0;
      laserMsgs.angle_max = +sensor.fov/2.0;
      laserMsgs.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
      laserMsgs.range_min = sensor.range.min;
      laserMsgs.range_max = sensor.range.max;
      laserMsgs.ranges.resize(sensor.ranges.size());
      laserMsgs.intensities.resize(sensor.intensities.size());

      collision = false;
      minFrontDist = sensor.range.max;
      // added by sepehr for random position init:
      //        double min_laser_val = 99;
      for(unsigned int i = 0; i < sensor.ranges.size(); i++)
        {
          laserMsgs.ranges[i] = sensor.ranges[i];
          if(sensor.ranges[i] < 0.6)
            collision = true;
          if( i > (sensor.fov*180.0/M_PI - 45)/2 && i < (sensor.fov*180.0/M_PI + 45)/2 && sensor.ranges[i]  < minFrontDist)
            minFrontDist = sensor.ranges[i];
          //            if(sensor.ranges[i] < min_laser_val)
          //                min_laser_val = sensor.ranges[i];
          laserMsgs.intensities[i] = sensor.intensities[i];
        }

      //        if( min_laser_val > 3.3 && rand()/float(RAND_MAX) > 0.1)
      //        {
      //            initial_poses.clear();
      //            initial_poses.push_back(robotmodel->positionmodel->GetGlobalPose());
      //        }
      laserMsgs.header.stamp = ros::Time::now();
      rosLaserData = laserMsgs;
    }
  if( minFrontDist > 3.6 && (double(rand())/RAND_MAX) > 0.1)
    robot->resetPose = robot->pos->GetPose();

  if( robot->pos->GetWorld()->SimTimeNow() - stgSpeedTime > 100000)
    robot->pos->SetSpeed( 0, 0, 0);


  //temp, just to check publish rate:
  if( allowNewMsg
      && laserMsgs.header.stamp > lastSentTime
      && rosCurPose.header.stamp > lastSentTime)
    {
      allowNewMsg = false;
      dqn_stage_ros::stage_message msg;
      msg.header.stamp = ros::Time::now();
      msg.collision = collision;
      msg.minFrontDist = minFrontDist;
      msg.position = rosCurPose;
      msg.laser = rosLaserData;
      pub_state_.publish( msg);
    }
}

void rosVelocityCB( const geometry_msgs::TwistConstPtr vel)
{
// ROS_WARN("Vel recieved");
  robot->pos->SetXSpeed( vel->linear.x);
  robot->pos->SetTurnSpeed( vel->angular.z);
  lastSentTime = ros::Time::now();
  stgSpeedTime = robot->pos->GetWorld()->SimTimeNow();
  allowNewMsg = true;
}

bool rosResetSrvCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose);
  return true;
}


extern "C" int Init( Model* mod )
{ 
  int argc = 0;
  char** argv;
  ros::init( argc, argv, "target_controller_node");
  n = new ros::NodeHandle();
  lastSentTime = ros::Time::now();
  pub_state_ = n->advertise<dqn_stage_ros::stage_message>("input_data", 15);
  sub_vel_ = n->subscribe( "cmd_vel", 15, &rosVelocityCB);
  reset_srv_ = n->advertiseService("reset_positions", &rosResetSrvCB);
  robot = new ModelRobot;
  robot->pos = (ModelPosition*) mod;
  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
  robot->pos->Subscribe();
  robot->resetPose = robot->pos->GetPose();
  //    robot->pos->GetChild("ranger:0")->Subscribe();
  robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
  robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
  robot->laser->Subscribe();
  return 0; //ok
}

