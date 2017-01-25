#include <stage.hh>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dqn_stage_ros/stage_message.h>

using namespace Stg;

struct ModelRobot
{
    ModelPosition* pos;
    ModelRanger* laser;
};

ModelRobot* robot;

ros::NodeHandle* n;
ros::Publisher pub_state_;

geometry_msgs::Pose rosCurPose;
sensor_msgs::LaserScan rosLaserData;
bool collision = false;
double minFrontDist;

void stgPoseUpdateCB( Model* mod, ModelRobot* robot)
{
    geometry_msgs::PoseStamped positionMsg;
    positionMsg.pose.position.x = robot->pos->pose.x;
    positionMsg.pose.position.y = robot->pos->pose.y;
    positionMsg.pose.position.z = robot->pos->pose.z;
//    positionMsg.pose.orientation
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
            if(sensor.ranges[i] < 0.3)
                collision = true;
            if( i > (sensor.fov - 45)/2 && i < (sensor.fov + 45)/2 && sensor.ranges[i]  < minFrontDist)
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
}

extern "C" int Init( Model* mod )
{ 
    int argc = 0;
    char** argv;
    ros::init( argc, argv, "target_controller_node");
    n = new ros::NodeHandle();
    pub_state_ = n->advertise<dqn_stage_ros::stage_message>("input_data", 15);

    robot = new ModelRobot;
    robot->pos = (ModelPosition*) mod;
    robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
    robot->pos->Subscribe();
    //    robot->pos->GetChild("ranger:0")->Subscribe();
    robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
    robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
    robot->laser->Subscribe();
    return 0; //ok
}

