// ROS
#include <ros/ros.h>
// Server
#include "optimize_planner/PathPlan.h"
// Customize optimize class

//#include "optimize_planner/include/motoman_moveit.h"
#include "optimize_planner.hpp"


// output
#include <iostream>

using namespace std;
motoman_move_group* m_robot_model;
EuclidianDis_optimize_planner* m_planner;

bool plan(optimize_planner::PathPlan::Request &req, optimize_planner::PathPlan::Response &res)
{

    std::string planning_arm = req.group_name;
    std::vector<double> goal_test(7,0.1);
    // Using Current configuration as start
    m_planner->start_Config = m_robot_model->GetGroupConfig(planning_arm);
    // Assign Goal Configuration
    m_planner->goal_Config = goal_test;
    m_planner->planning_time = 5;

    m_planner->start_planning();

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimize_plan_server");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("Optimize_plan_server",plan);

    m_robot_model = new motoman_move_group;

    ROS_INFO("Robot Model Loaded!");
    m_planner = new EuclidianDis_optimize_planner;
    ROS_INFO("Optimize planner created!");

    ROS_INFO(">>>>>>>>>>>>>>> Plan server created! >>>>>>>>>>>>>>>");

    ros::spin();
    return 0;
}
