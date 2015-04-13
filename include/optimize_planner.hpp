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

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// robot model
#include "motoman_moveit.hpp"
//#include "optimize_planner/include/euclidian_se3_cost.hpp"

// Cost Function
//#include "euclidian_se3_cost.hpp"

// Trajetory
#include <moveit/robot_trajectory/robot_trajectory.h>

enum planner_type
{
    RRT_STAR,
    RRT_Connect
};

#define WorkSpace_X_Limits 1.5 // meters
#define WorkSpace_Y_Limits 2.0 // meters
#define WorkSpace_Z_Limits 2.0 // meters

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SE3dis_OptimizationObjective : public ompl::base::OptimizationObjective
{
public:
    SE3dis_OptimizationObjective(const ompl::base::SpaceInformationPtr &si) : ompl::base::OptimizationObjective(si){}
    virtual ~SE3dis_OptimizationObjective(){}

    virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;

};

namespace optimize_planner
{
    class motoman_planner
    {
        public:
             motoman_planner()
             {
                // Give default planning time
                planning_time = 10;
                goal_tolerance = 0.01;
                cost_bias = 0.5;
                planner_choice = RRT_STAR;
             }

             bool start_planning()
            {
                ob::StateSpacePtr Joint_space(new ob::RealVectorStateSpace(motoman_arm_DOF));
                ob::RealVectorBounds Joint_bounds(motoman_arm_DOF);

                //Joint_bounds()
                setStateSpaceLimits(Joint_bounds);
                Joint_space->as<ob::RealVectorStateSpace>()->setBounds(Joint_bounds);

                // Setup Sample space
                ob::SpaceInformationPtr sample_si(new ob::SpaceInformation(Joint_space));

                // Setup Collision Checker, Haven't finished yet
                //sample_si->setStateValidityChecker(boost::bind(&motoman_planner::isStateValid,_1));

                ob::ScopedState<> start_state = SetStateConfig(start_Config,sample_si);
                start_state.print(std::cout);
                ob::ScopedState<> goal_state= SetStateConfig(goal_Config,sample_si);
                goal_state.print(std::cout);

                // Creat Problem Definition
                ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(sample_si));
                pdef->setStartAndGoalStates(start_state,goal_state);

                // Make a cost function that combines path length with minimizing Euclidian cost
                ob::MultiOptimizationObjective* combined_cost_fn = new ob::MultiOptimizationObjective(sample_si);
                ob::OptimizationObjectivePtr path_length_cost_fn(new ob::PathLengthOptimizationObjective(sample_si));
                SE3dis_OptimizationObjective m_cost_fn(sample_si);
                ob::OptimizationObjectivePtr workspace_cost_fn(&m_cost_fn);

                combined_cost_fn->addObjective(path_length_cost_fn, (1.0 - cost_bias));
                combined_cost_fn->addObjective(workspace_cost_fn, cost_bias);

                ompl::base::OptimizationObjectivePtr cost_fn(combined_cost_fn);
                pdef->setOptimizationObjective(cost_fn);

                ROS_INFO("Cost Function Has been Set");

                // Make the planner
                ob::PlannerPtr planner(new og::RRTstar(sample_si));

                planner->setProblemDefinition(pdef);
                planner->setup();
                // Plan!!
                ob::PlannerStatus solved = planner->solve(planning_time);
                if(solved)
                {
                    // Generate the Path
                    path = pdef->getSolutionPath();
                    std::cout<<"Find Path"<<std::endl;
                    path->print(std::cout);
                    return true;
                }
                else
                {
                    std::cout<<"Can't find solution"<<std::endl;
                    return false;
                }
            }

        private:

             bool setStateSpaceLimits(ob::RealVectorBounds& Joint_bounds)
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

                 return true;
             }

             ob::ScopedState<> SetStateConfig(std::vector<double>& state_config, ob::SpaceInformationPtr smp_space)
             {
                 ob::ScopedState<> rt_state(smp_space);
                 if(state_config.size())
                 {
                     std::vector<double>::iterator iter = state_config.begin();
                     for(int i = 0;iter!= state_config.end();iter++,i++)
                     {
                         rt_state->as<ob::RealVectorStateSpace::StateType>()->values[i] = *iter;
                     }
                 }

                 return rt_state;
             }
             bool isStateValid(const ob::State* state)
             {
                 // Moveit is needed here
                 const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
                 const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
                 const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                 return (const void*)rot != (const void*)pos;
             }

        public:
             float planning_time;
             double goal_tolerance;
             planner_type planner_choice;
             float cost_bias;

             std::vector<double> start_Config;
             std::vector<double> goal_Config;

             ob::PathPtr path;
    };
}


