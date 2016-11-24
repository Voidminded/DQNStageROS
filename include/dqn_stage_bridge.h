#ifndef DQN_STAGE_BRIDGE_H
#define DQN_STAGE_BRIDGE_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>

#define PI 3.14159265359

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

  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  image_transport::ImageTransport it;
  image_transport::Publisher pub_laser_iamge_;

  virtual void SpinOnce();

  void generateOccupancyGridCallBack(const  sensor_msgs::LaserScanConstPtr& scan);
  void updateRobotPoseCallBack(const  nav_msgs::OdometryConstPtr& odom);
};

#endif 
