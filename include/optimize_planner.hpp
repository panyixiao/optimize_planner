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
//#include "optimize_planner/include/euclidian_se3_cost.hpp"

// Cost Function
#include "euclidian_se3_cost.hpp"

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
            cost_bias = 0.1;
         }

        bool start_planning(std::string group_name)
        {
            std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

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
            //pdef->addStartState(start_state);

            // Make a cost function that combines path length with minimizing workspace Euclidian cost
            ob::MultiOptimizationObjective* combined_cost_fn = new ob::MultiOptimizationObjective(sample_si);
            ob::OptimizationObjectivePtr path_length_cost_fn(new ob::PathLengthOptimizationObjective(sample_si));
            ob::OptimizationObjectivePtr workspace_cost_fn(new optimize_planner::SE3dis_OptimizationObjective(sample_si,&m_robot_model,group_name));

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
            ob::PlannerStatus status = planner->solve(planning_time);

            // Generate a trajectory

            if(status == ob::PlannerStatus::EXACT_SOLUTION || status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
            {
                std::cout<<"Find Path"<<std::endl;
                // Generate the Path
                optimized_trajectory.clear();

                path = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());

                int interpolate_num = 5*round(path->length());

                path->interpolate(interpolate_num);

                std::vector<ob::State*> state = path->getStates();

                for(int i = 0; i<state.size(); i++)
                {
                    ob::State* Jnt_config = state[i];
                    std::vector<double> jnt_config(motoman_arm_DOF,0);

                    for(int j = 0; j<motoman_arm_DOF;j++)
                    {
                        jnt_config[j] = Jnt_config->as<ob::RealVectorStateSpace::StateType>()->values[j];
                    }
                    optimized_trajectory.push_back(jnt_config);
                }

                path_length = calculate_trajectory_euclidean_length(group_name);
                path_cost = path->cost(cost_fn).value();

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

         bool Create_trajectory(boost::shared_ptr<og::PathGeometric> path_solution)
         {
            std::vector<ob::State*> states = path_solution->getStates();

         }

         bool isStateValid(const ob::State* state)
         {
             // Moveit is needed here
             const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
             const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
             const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

             return (const void*)rot != (const void*)pos;
         }

         double calculate_trajectory_euclidean_length(std::string& group_name)
         {
            double length = 0.0;
            if(optimized_trajectory.empty())
            {
                ROS_INFO("Empty Trajectory!");
                return length;
            }
            int i = 0;
            int j = i+1;

            for( ;j<optimized_trajectory.size(); i++,j++)
            {
                std::vector<double> config_1 = optimized_trajectory[i];
                std::vector<double> config_2 = optimized_trajectory[j];

                double temp = m_robot_model.get_euclidean_distance(group_name,config_1,config_2);

                length += temp;
            }

            return length;
         }

    public:
         double path_length;
         double path_cost;

         float planning_time;
         double goal_tolerance;
         float cost_bias;

         motoman_move_group m_robot_model;

         std::vector<double> start_Config;
         std::vector<double> goal_Config;

         std::vector< std::vector<double> > optimized_trajectory;

    private:
         boost::shared_ptr<og::PathGeometric> path;
    };

}


