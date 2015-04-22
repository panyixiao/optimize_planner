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
    // Goal 1
    // -1.5442344278755054, 1.0688537862129166, 0.9596692482080396, -1.40904847193336, 1.7674462114208445, 0.6331007451974301, -1.031333210948469
    // -0.8948681741005134, 1.8402361216991319, -2.817438019916313, -1.5863995559308288, 2.266365227148054, -1.6504599824522053, 0.11104174698980832

    // -0.6811055960290769, 0.9630132566179345, -0.9416440752119656, -1.5486826273467296, 0.9426491731441866, 1.8999759259764337, -1.302248941685609

    float Jnt_Val_s = -0.6811055960290769;
    float Jnt_Val_l = 0.9630132566179345;
    float Jnt_Val_e = -0.9416440752119656;
    float Jnt_Val_u = -1.5486826273467296;
    float Jnt_Val_r = 0.9426491731441866;
    float Jnt_Val_b =  1.8999759259764337;
    float Jnt_Val_t =  -1.302248941685609;

//    float Jnt_Val_s =  1.3507845631700357;
//    float Jnt_Val_l = -1.6832280657563559;
//    float Jnt_Val_e = -2.2392413597977727;
//    float Jnt_Val_u = -1.3039921351861266;
//    float Jnt_Val_r =  2.1722310048916844;
//    float Jnt_Val_b =  0.8222022963902568;
//    float Jnt_Val_t =  0.5076985475703006;

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

