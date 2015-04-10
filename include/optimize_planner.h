// ROS
#include <ros/ros.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>


enum planner_selection
{
    RRT_STAR,
    RRT_Connect
};

// Manipulator DOF
#define arm_DOF 7
// motoman_joint_limits
#define Joint_1_s_Limits 3.13
#define Joint_2_l_Limits 1.90
#define Joint_3_e_Limits 2.95
#define Joint_4_u_Limits 2.36
#define Joint_5_r_Limits 3.13
#define Joint_6_b_Limits 1.90
#define Joint_7_t_Limits 3.13
#define WorkSpace_X_Limits 1.5 // meters
#define WorkSpace_Y_Limits 2.0 // meters
#define WorkSpace_Z_Limits 2.0 // meters

namespace ob = ompl::base;
namespace og = ompl::geometric;

int Ajay;

class optimize_planner
{
public:
     optimize_planner()
     {
        Joint_space = new ob::RealVectorStateSpace(arm_DOF);
        Joint_bounds = ob::RealVectorBounds(arm_DOF);
        setStateSpaceLimits();
        sample_si = ob::SpaceInformation(Joint_space);
        // Haven't finished yet
        sample_si->setStateValidityChecker(boost::bind(&isStateValid,_1));

        start = ob::ScopedState<>(sample_si);
        goal = ob::ScopedState<>(sample_si);
        pdef = new ob::ProblemDefinition(sample_si);
     }

    bool planner_select(planner_selection planner_type)
    {
        switch(planner_type)
        {
        case RRT_STAR:
            planner = new og::RRTstar(sample_si);
        case RRT_Connect:
            planner = new og::RRTConnect(sample_si);
        default:
            ROS_INFO("Can't find such type of planner!");
            return false;
        }
        return true;
    }

    bool Init_start_state()
    {

        return true;
    }

    bool Set_goal_state()
    {
        return true;
    }

    bool start_planning()
    {
        if(Init_start_state() && Set_goal_state())
        {
            pdef->setStartAndGoalStates(start,goal);
            planner->setProblemDefinition(pdef);
            planner->setup();

            ob::PlannerStatus solved = planner->solve(planning_time);
            if(solved)
            {
                ob::PathPtr path = pdef->getSolutionPath();
                std::cout<<"Find Path"<<std::endl;
                path->print(std::cout);
            }
            else
            {
                std::cout<<"Can't find"<<std::endl;
            }
        }

        else
        {
            ROS_INFO("Initial / Goal state invalid!");
            return false;
        }
    }
private:
     void setStateSpaceLimits()
     {
         // JointSpace Limits
         Joint_bounds.setLow(0,-Joint_1_s_Limits);
         Joint_bounds.setLow(1,-Joint_2_l_Limits);
         Joint_bounds.setLow(2,-Joint_3_e_Limits);
         Joint_bounds.setLow(3,-Joint_4_u_Limits);
         Joint_bounds.setLow(4,-Joint_5_r_Limits);
         Joint_bounds.setLow(5,-Joint_6_b_Limits);
         Joint_bounds.setLow(6,-Joint_7_t_Limits);

         Joint_bounds.setHigh(0,Joint_1_s_Limits);
         Joint_bounds.setHigh(1,Joint_2_l_Limits);
         Joint_bounds.setHigh(2,Joint_3_e_Limits);
         Joint_bounds.setHigh(3,Joint_4_u_Limits);
         Joint_bounds.setHigh(4,Joint_5_r_Limits);
         Joint_bounds.setHigh(5,Joint_6_b_Limits);
         Joint_bounds.setHigh(6,Joint_7_t_Limits);

         Joint_space->as<ob::RealVectorStateSpace>()->setBounds(Joint_bounds);
     }

     bool isStateValid(const ob::State* state)
     {
         // Moveit is needed here
        return true;
     }

public:
     float planning_time;
     ob::ScopedState<> start;
     ob::ScopedState<> goal;

private:
     ob::SpaceInformationPtr sample_si;

     ob::StateSpacePtr Joint_space;
     ob::RealVectorBounds Joint_bounds;

     ob::ProblemDefinitionPtr pdef;
     ob::PlannerPtr planner;

}
