// ROS
#include <ros/ros.h>
// Server
#include "optimize_planner/PathPlan.h"
// Customize optimize class
//#include <optimize_planner/include/optimize_planner.h>


// output
#include <iostream>

using namespace std;

bool plan(optimize_planner::PathPlan::Request &req, optimize_planner::PathPlan::Response &res)
{
    // Setup Space Information

    // Setup Collision Checker

    // Creat Problem Definition

    // Define Cost Function

    // Make the planner

    // Plan!!

    // Generate the Path
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimize_plan_server");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("Optimize_plan_server",plan);
    ROS_INFO("Optimize plan server created!");
    ros::spin();
    return 0;
}
