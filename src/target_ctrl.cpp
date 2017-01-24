#include <stage.hh>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

using namespace Stg;
Model* target;
ros::NodeHandle* n;
ros::Subscriber sub;

void updateCB(const geometry_msgs::PoseStampedConstPtr &pose)
{
    ROS_WARN("ROS Sub");
    Pose p;
    p.x = pose->pose.position.x;
    p.y = pose->pose.position.y;
    p.z = 0;
    target->SetPose( p);
    return;
}

int spinCB( Model* mod, ros::NodeHandle* dummy)
{
    ros::spinOnce();
    return 0;
}


extern "C" int Init( Model* mod )
{ 
	int argc = 0;
	char** argv; 
	ros::init( argc, argv, "target_controller_node");
    n = new ros::NodeHandle();
    sub = n->subscribe("/target", 15, &updateCB);
    target = /*(ModelPosition* )*/mod;

  // attach callback to be called whenever a flag is added to the sink model 
    target->AddCallback( Model::CB_UPDATE, (model_callback_t)spinCB, NULL );
    target->Subscribe();
//    ros::spin();
    return 0; //ok
}

