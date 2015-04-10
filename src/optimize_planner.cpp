// ROS
#include <ros/ros.h>
// Server
#include "optimize_planner/PathPlan.h"


#include <ompl/geometric/planners/rrt/RRTstar.h>

// Customize optimize class
#include "optimize_planner/include/optimize_planner.h"

// output
#include <iostream>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool plan(optimize_planner::PathPlan::Request &req, optimize_planner::PathPlan::Response &res)
{
    ob::StateSpacePtr Joint_space(new ob::RealVectorStateSpace(7));
    ob::StateSpacePtr World_space(new ob::SE3StateSpace());
    ob::RealVectorBounds Joint_bounds(7);
    ob::RealVectorBounds Worldspace_bounds(3);

    // Setup Space Information

    Worldspace_bounds.setLow(0,-WorkSpace_X_Limits);
    Worldspace_bounds.setHigh(0,-WorkSpace_X_Limits);
    Worldspace_bounds.setLow(1,-WorkSpace_Y_Limits);
    Worldspace_bounds.setHigh(1,-WorkSpace_Y_Limits);
    Worldspace_bounds.setLow(2,-WorkSpace_Z_Limits);
    Worldspace_bounds.setHigh(2,-WorkSpace_Z_Limits);

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
