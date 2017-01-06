#ifndef DQN_STAGE_BRIDGE_H
#define DQN_STAGE_BRIDGE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include <opencv2/opencv.hpp>

#define PI 3.14159265359

#define Map_Max_Dist 40.0

using namespace cv;

class DQNStageBridge{
public:
  DQNStageBridge(ros::NodeHandle& nh);
	virtual void Spin();
protected:
  ros::NodeHandle nh_;
private:
  Mat laserMap;
  geometry_msgs::Pose robotPose;
  geometry_msgs::Pose goalPose;
  std_msgs::Float32 robotRot;
  double dirToTarget;

  ros::Subscriber sub_laser_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_action_;
  ros::Subscriber sub_new_goal_;
  ros::Subscriber sub_last_reward_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_cur_pose_;
  ros::Publisher pub_cur_rot_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_impact_;
  image_transport::ImageTransport it;
  image_transport::Publisher pub_laser_image_;

  virtual void SpinOnce();

  void selectGoalCB(const std_msgs::EmptyConstPtr& msg);
  void generateOccupancyGridCB(const  sensor_msgs::LaserScanConstPtr& scan);
  void updateRobotPoseCB(const  nav_msgs::OdometryConstPtr& odom);
  void actionGeneratorCB( const std_msgs::Int8ConstPtr& act);
  void printRewardCB( const std_msgs::Float32ConstPtr& rwd);

  static double constrainAngle(double x);
};

#endif 
