#include "dqn_stage_bridge.h"

DQNStageBridge::DQNStageBridge(ros::NodeHandle& nh)
  :nh_(nh)
{

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
