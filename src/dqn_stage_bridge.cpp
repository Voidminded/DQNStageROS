#include "dqn_stage_bridge.h"
//#include "include/dqn_stage_bridge.h"

DQNStageBridge::DQNStageBridge(ros::NodeHandle& nh)
  :nh_(nh),
    it(nh_),
    sub_laser_(nh_.subscribe("base_scan", 1, &DQNStageBridge::generateOccupancyGridCB, this)),
    sub_odom_(nh_.subscribe("base_pose_ground_truth", 1, &DQNStageBridge::updateRobotPoseCB, this)),
    sub_action_(nh_.subscribe("dqn/selected_action", 10, &DQNStageBridge::actionGeneratorCB, this)),
    pub_goal_(nh_.advertise<geometry_msgs::Pose>("bridge/goal_pose", 1, true)),
    pub_cur_pose_(nh_.advertise<geometry_msgs::Pose>("bridge/current_pose", 1, true)),
    pub_cur_rot_(nh_.advertise<std_msgs::Float32>("bridge/current_direction", 1, true)),
    pub_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true))
{
  pub_laser_image_ = it.advertise("bridge/laser_image", 1);
  laserMap.create( 120, 120, CV_8U);
}

void DQNStageBridge::generateOccupancyGridCB(const sensor_msgs::LaserScanConstPtr &scan)
{
  if( scan->ranges.size() < 1)
    return;
  else
    {
      laserMap = Scalar( 128, 128, 128);
      int x = scan->range_max* 10, y = scan->range_max*10, rad = scan->range_max*10;
      int dir = (int)robotRot;
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
      pub_laser_image_.publish( laserImageMSG);
    }
}

void DQNStageBridge::updateRobotPoseCB(const nav_msgs::OdometryConstPtr &odom)
{
  robotPose = odom->pose.pose;
  tf::Pose p;
  tf::poseMsgToTF(robotPose, p);
  robotRot = 180*tf::getYaw(p.getRotation())/PI;
  //ROS_INFO("[BRDG] Rboto orientation: %g", robotRot);
}

/********************************************
 * Considering 3 forward speed, 3 rotational speed for each direction, + stop for each
 * Forward: [0, 0.3, 0.6, 0.9]
 * Rotational: [-0.9, -0.6, -0.3, 0, 0.3, 0.6, 0.9]
 * Thus action space is 4*7=28 actions [0, 27]
 * ******************************************/
void DQNStageBridge::actionGeneratorCB(const std_msgs::Int8ConstPtr &act)
{
  if( act->data < 0 || act->data > 27)
  {
    ROS_WARN("Invalid action %d", act->data);
    return;
  }
  double w = ((act->data%7)-3)/3.0;
  double f = 0.3*(act->data/7);
  geometry_msgs::Twist msg;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = w;
  msg.linear.x = f;
  msg.linear.y = 0;
  msg.linear.z = 0;
  pub_cmd_vel_.publish( msg);
  //ROS_INFO("Action %d received, x: %g w: %g", act->data, f, w);
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
  ros::init(argc, argv, "dqn_stage_bridge_node");
  ros::NodeHandle nh;
  DQNStageBridge bridge_node(nh);
  ROS_INFO("[BRDG] Bridge node initialized...");
  bridge_node.Spin();
  return 0;
}
