#include "dqn_stage_bridge.h"
//#include "include/dqn_stage_bridge.h"

DQNStageBridge::DQNStageBridge(ros::NodeHandle& nh)
  :nh_(nh),
    it(nh_),
    laser_sub_(nh_.subscribe("base_scan", 30, &DQNStageBridge::generateOccupancyGridCallBack, this)),
    odom_sub_(nh_.subscribe("base_pose_ground_truth", 30, &DQNStageBridge::updateRobotPoseCallBack, this))
{
  pub_laser_iamge_ = it.advertise("laser_image", 1);
  laserMap.create( 120, 120, CV_8U);
}

void DQNStageBridge::generateOccupancyGridCallBack(const sensor_msgs::LaserScanConstPtr &scan)
{
  if( scan->ranges.size() < 1)
    return;
  else
    {
      laserMap = Scalar( 128, 128, 128);
      int x = scan->range_max* 10, y = scan->range_max*10, rad = scan->range_max*10;
      tf::Pose p;
      tf::poseMsgToTF(robotPose, p);
      int dir = 180*tf::getYaw(p.getRotation())/PI;
      for( float ind = 0; ind < 360; ind++)
        {
          int i = ind + dir;
          for( int j = 0; j < rad; j++)
            if( scan->ranges.at(ind)*10>=j)
              laserMap.data[ (int)scan->range_max *20 *(x+ int( j*cos( PI*i/180))) + y + int( j*sin( PI*i/180))] = 255;
          if( scan->ranges.at(ind) < 6)
            laserMap.data[ (int)scan->range_max*20 *(x+ int( 10*scan->ranges.at(ind)*cos( PI*i/180))) + y + int( 10*scan->ranges.at(ind)*sin( PI*i/180))] = 0;
        }
      sensor_msgs::ImagePtr laserImageMSG = cv_bridge::CvImage(std_msgs::Header(), "mono8", laserMap).toImageMsg();
      pub_laser_iamge_.publish( laserImageMSG);
    }
}

void DQNStageBridge::updateRobotPoseCallBack(const nav_msgs::OdometryConstPtr &odom)
{
  robotPose = odom->pose.pose;
}

void DQNStageBridge::SpinOnce()
{
  ros::spinOnce();
}

void DQNStageBridge::Spin()
{
  //ros::Rate rate(update_rate_);
  while (ros::ok())
    {
      SpinOnce();
      // TODO: Graceful shutdown:
      // http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
      // if (!rate.sleep())
      // {
      //   ROS_WARN_STREAM("[ANIM] missed target loop rate of " << update_rate_ << " hz.");
      // }
    }

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dqn_stage_bridge");
  ros::NodeHandle nh;
  DQNStageBridge bridge_node(nh);
  ROS_INFO("[BRDG] Bridge node initialized...");
  bridge_node.Spin();
  return 0;
}
