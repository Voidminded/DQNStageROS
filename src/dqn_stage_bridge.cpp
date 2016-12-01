#include "dqn_stage_bridge.h"
//#include "include/dqn_stage_bridge.h"

DQNStageBridge::DQNStageBridge(ros::NodeHandle& nh)
  :nh_(nh),
    it(nh_),
    sub_laser_(nh_.subscribe("base_scan", 1, &DQNStageBridge::generateOccupancyGridCB, this)),
    sub_odom_(nh_.subscribe("base_pose_ground_truth", 1, &DQNStageBridge::updateRobotPoseCB, this)),
    sub_action_(nh_.subscribe("dqn/selected_action", 1, &DQNStageBridge::actionGeneratorCB, this)),
    sub_new_goal_(nh_.subscribe("dqn/new_goal", 1, &DQNStageBridge::selectGoalCB, this)),
    pub_goal_(nh_.advertise<geometry_msgs::Pose>("bridge/goal_pose", 1, true)),
    pub_cur_pose_(nh_.advertise<geometry_msgs::Pose>("bridge/current_pose", 1, true)),
    pub_cur_rot_(nh_.advertise<std_msgs::Float32>("bridge/current_direction", 1, true)),
    pub_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true)),
    pub_impact_(nh_.advertise<std_msgs::Empty>("bridge/impact", 1, true))
{
  pub_laser_image_ = it.advertise("bridge/laser_image", 1);
  laserMap.create( 80, 80, CV_8U);
}

double DQNStageBridge::constrainAngle(double x)
{
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

void DQNStageBridge::generateOccupancyGridCB(const sensor_msgs::LaserScanConstPtr &scan)
{
  if( scan->ranges.size() < 1)
    return;
  else
    {
      laserMap = Scalar( 128, 128, 128);
      int x = scan->range_max* 10, y = scan->range_max*10, rad = scan->range_max*10;
      int dir = 0 ;//(int)robotRot.data;
      bool boomed = false;
      for( float ind = 0; ind < 360; ind++)
      {
        int i = ind + dir;
        for( int j = 0; j < rad; j++)
          if( scan->ranges.at(ind)*10>=j)
            laserMap.data[ 80 *(x+ int( j*cos( PI*i/180))) + y + int( j*sin( PI*i/180))] = 255;
        if( scan->ranges.at(ind) < scan->range_max)
          laserMap.data[ 80 *(x+ int( 10*scan->ranges.at(ind)*cos( PI*i/180))) + y + int( 10*scan->ranges.at(ind)*sin( PI*i/180))] = 0;
        if( scan->ranges.at(ind) < 0.66)
          boomed = true;
      }
      if( boomed)
      {
        std_msgs::Empty boom;
        pub_impact_.publish( boom);
      }
      for( int i = rad*2; i < 80; i++)
      {
        int stir = (dirToTarget*40/180)+40;
        int minStir = min( 40, stir);
        int maxStir = max( 40, stir);
        for( int j = 0; j < 80; j++)
        {
          if( j >= minStir && j <= maxStir)
            laserMap.at<uchar>(i,j) = 0;
          else
            laserMap.at<uchar>(i,j) = 255;
        }
      }
      for( int j = rad*2; j < 80; j++)
      {
        double dist = sqrt( pow(robotPose.position.x - goalPose.position.x, 2) + pow( robotPose.position.y - goalPose.position.y, 2));
        int normDist = 72 * dist / Map_Max_Dist;
        for( int i = 0; i < 72; i++)
        {
          if( i <= normDist)
            laserMap.at<uchar>(i,j) = 0;
          else
            laserMap.at<uchar>(i,j) = 255;
        }
      }
      sensor_msgs::ImagePtr laserImageMSG = cv_bridge::CvImage(std_msgs::Header(), "mono8", laserMap).toImageMsg();
      pub_laser_image_.publish( laserImageMSG);
    }
}

void DQNStageBridge::updateRobotPoseCB(const nav_msgs::OdometryConstPtr &odom)
{
  robotPose = odom->pose.pose;
  tf::Pose pos;
  tf::poseMsgToTF(robotPose, pos);
  robotRot.data = 180.0*tf::getYaw(pos.getRotation())/PI;
  double angleBetween = 180.0*atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x)/PI;
  dirToTarget = DQNStageBridge::constrainAngle(robotRot.data - angleBetween);
//  ROS_INFO("[BRDG] Rboto orientation: %g angle between: %g", robotRot.data, angleBetween);
//  ROS_INFO("[BRDG] Dir to Target: %g", dirToTarget);
  pub_cur_pose_.publish( robotPose);
  pub_cur_rot_.publish( robotRot);
  pub_goal_.publish( goalPose);
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

void DQNStageBridge::selectGoalCB(const std_msgs::EmptyConstPtr &msg)
{
  double theta = double(rand())/RAND_MAX;
  double r = rand()%21;
  goalPose.position.x = r*cos(theta);
  goalPose.position.y = r*sin(theta);
  ROS_INFO("Selected new goal : %g, %g", goalPose.position.x, goalPose.position.y);
  pub_goal_.publish( goalPose);
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
