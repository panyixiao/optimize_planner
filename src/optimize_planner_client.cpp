#include <ros/ros.h>
#include "optimize_planner/PathPlan.h"

// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Move Group
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <sstream>


using namespace move_group_interface;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimize_Planning_Client");

    ros::NodeHandle nh;
    ros::ServiceClient Client = nh.serviceClient<optimize_planner::PathPlan>("Optimize_plan_server");
    optimize_planner::PathPlan srv;

    srv.request.group_name = "arm_left";

    // -0.3772088970063918, -1.3143374665077188, 1.3055896989571494, -1.2431863366129394, -2.9914519460193256, 1.3527307468837322, 1.6504463451922884
    float Jnt_Val_s = -0.3772088970063918;
    float Jnt_Val_l = -1.3143374665077188;
    float Jnt_Val_e =  1.3055896989571494;
    float Jnt_Val_u = -1.2431863366129394;
    float Jnt_Val_r = -2.9914519460193256;
    float Jnt_Val_b =  1.3527307468837322;
    float Jnt_Val_t =  1.6504463451922884;


    srv.request.target_config.push_back(Jnt_Val_s);
    srv.request.target_config.push_back(Jnt_Val_l);
    srv.request.target_config.push_back(Jnt_Val_e);
    srv.request.target_config.push_back(Jnt_Val_u);
    srv.request.target_config.push_back(Jnt_Val_r);
    srv.request.target_config.push_back(Jnt_Val_b);
    srv.request.target_config.push_back(Jnt_Val_t);

    double m_time_limit = 10;
    double m_cost_weight = 0.7;

    if(argc>1)
    {
        //std::cout<<"Assigning planning time.."<<std::endl;
        std::istringstream iss(argv[1]);
        iss>>m_time_limit;
    }
    if(argc>2)
    {
        //std::cout<<"Assigning Cost weight.."<<std::endl;
        std::istringstream iss(argv[2]);
        iss>>m_cost_weight;
        if(m_cost_weight>1||m_cost_weight<0)
        {
            m_cost_weight = 0.5;
        }
    }

    std::cout<<"Planning Time: "<< m_time_limit;
    std::cout<<" | Cost Weight: "<<m_cost_weight<<std::endl;

    srv.request.time_limit = m_time_limit;
    srv.request.cost_weight = m_cost_weight;

    if(Client.call(srv))
    {
        //ROS_INFO("Found! Test Complete!");

        std::cout<<"Total Length is: "<< srv.response.total_length;
        std::cout<<" | Total Cost is: "<<srv.response.total_cost<<std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service!");
        return 1;
    }

    return 0;
}

