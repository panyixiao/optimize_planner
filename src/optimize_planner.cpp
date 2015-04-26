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
//motoman_move_group* m_robot_model;
optimize_planner::motoman_planner* m_planner;

bool plan(optimize_planner::PathPlan::Request &req, optimize_planner::PathPlan::Response &res)
{
    // Setting Start & Goal Configuration
    m_planner->start_Config.clear();
    m_planner->start_Config = m_planner->m_robot_model.GetGroupCurrentConfig(req.group_name);
    m_planner->goal_Config.clear();

    if(req.target_config.empty())
    {
        std::cout<< "No target assigned!, Exit!"<<std::endl;
        return false;
    }
    std::vector<double> temp_target_config;
    for(int i = 0;i<req.target_config.size();i++)
    {
        double JntValue = req.target_config[i];
        temp_target_config.push_back(JntValue);
    }

    m_planner->goal_Config = temp_target_config;
    // Setting Planning Time
    m_planner->planning_time = req.time_limit;
    // Setting Cost Bias
    m_planner->cost_bias = req.cost_weight;

    // Start Planning
    if(m_planner->start_planning(req.group_name))
    {
        res.total_cost = m_planner->path_cost;
        res.total_length = m_planner->path_length;

        // Use different color to display
        m_planner->m_robot_model.display_color_scale = req.cost_weight;
        // Generate & Execute/Display a trajectory
        bool Traj_generated = m_planner->m_robot_model.generate_trajectory(m_planner->optimized_trajectory,req.group_name);

        if(Traj_generated)
        {
            // Display Traj
            m_planner->m_robot_model.display_traj(m_planner->m_robot_model.m_trajectory,req.group_name);

            ROS_INFO("Sending Trajectory back to client");

            res.plan = m_planner->m_robot_model.m_trajectory;

            return true;
        }
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimize_plan_server");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("Optimize_plan_server",plan);

    m_planner = new optimize_planner::motoman_planner;

    ROS_INFO(">>>>>>>>>>>>>>> Plan server created! >>>>>>>>>>>>>>>");

    ros::spin();
    return 0;
}
