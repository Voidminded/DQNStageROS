#ifndef DQN_STAGE_BRIDGE_H
#define DQN_STAGE_BRIDGE_H

#include "ros/ros.h"

class DQNStageBridge{
public:
  DQNStageBridge(ros::NodeHandle& nh);
	virtual void Spin();
protected:
  ros::NodeHandle nh_;
private:
	virtual void SpinOnce();
};

#endif 
