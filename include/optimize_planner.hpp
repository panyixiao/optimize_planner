//ROS
#include <ros/ros.h>

//OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//OMPL Planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


//MOVEIT
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// robot model
//#include "optimize_planner/include/euclidian_se3_cost.hpp"

// Cost Function
#include "euclidian_se3_cost.hpp"

// Trajetory
#include <moveit/robot_trajectory/robot_trajectory.h>

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

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

void dealocate_StateValidityChecker_fn(ompl::base::StateValidityChecker* p)
{
    std::cout << ">>>>>>>>>>>>>>>Deallocate>>>>>>>>>>>>" << std::endl ;
    UNUSED(p);
}

void dealocate_MotionValidator_fn(ompl::base::MotionValidator* p)
{
    UNUSED(p);
}

void dealocate_OptimiztionObjective_fn(ompl::base::OptimizationObjective* p)
{
    UNUSED(p);
}


class ValidityChecker : public ob::StateValidityChecker
{
protected:

    std::unique_ptr<planning_scene::PlanningScene> planning_scene_ptr_;
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si)
    {
        robot_model_loader::RobotModelLoader model_loader("robot_description");
        planning_scene_ptr_.reset();
        planning_scene_ptr_ = std::unique_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(model_loader.getModel()));
        robot_model::RobotModelPtr r_model = model_loader.getModel();
        //std::cout << "variable count " << r_model->getVariableCount() << std::endl ;
        specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::NONE;
        specs_.hasValidDirectionComputation = false;
    }

    //Function to check if state is valid
    virtual bool isValid(const ob::State *state) const
    {
        //std::cout << ">>>>>>>>>>>>>>>Validity Check>>>>>>>>>>>>" << std::endl ;
        std::vector<double> JointValues(7) ;
        std::vector<std::string> variable_names(7) ;
        const ob::RealVectorStateSpace::StateType *sample_state = state->as<ob::RealVectorStateSpace::StateType>() ;

        robot_state::RobotState& current_state = planning_scene_ptr_->getCurrentStateNonConst();
        const robot_model::JointModelGroup* model_group = current_state.getJointModelGroup("arm_left");

        variable_names[0] = "arm_left_joint_1_s" ;
        JointValues[0] = sample_state->values[0] ;

        variable_names[1] = "arm_left_joint_2_l" ;
        JointValues[1] = sample_state->values[1] ;

        variable_names[2] = "arm_left_joint_3_e" ;
        JointValues[2] = sample_state->values[2] ;

        variable_names[3] = "arm_left_joint_4_u" ;
        JointValues[3] = sample_state->values[3] ;

        variable_names[4] = "arm_left_joint_5_r" ;
        JointValues[4] = sample_state->values[4] ;

        variable_names[5] = "arm_left_joint_6_b" ;
        JointValues[5] = sample_state->values[5] ;

        variable_names[6] = "arm_left_joint_7_t" ;
        JointValues[6] = sample_state->values[6] ;

        current_state.setVariablePositions(variable_names,JointValues) ;

        //std::cout << *(current_state.getJointPositions("arm_left_joint_2_l")) << std::endl ;

        //std::cout << "state valid ? " << planning_scene_ptr_->isStateValid(current_state, "left_arm") << std::endl ;
        //std::cout << "state valid ? " << planning_scene_ptr_->isStateColliding(current_state,"left_arm") << std::endl ;
        //std::cout << "state bound ? " << current_state.satisfiesBounds(model_group) << std::endl ;

        if(planning_scene_ptr_->isStateValid(current_state, "left_arm") == 1
                && current_state.satisfiesBounds(model_group) == 1
                && planning_scene_ptr_->isStateColliding(current_state,"left_arm") == 0)
        {
            std::cout << ">>>>>>>>>>>>>>>Valid State>>>>>>>>>>>>" << std::endl ;
            return true  ;
        }
        else
        {
            std::cout << ">>>>>>>>>>>>>>>Invalid State>>>>>>>>>>>>" << std::endl ;
            return false  ;
        }
    }

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
            cost_bias = 0.1;
            planner_choice = RRT_STAR;
         }

         bool start_planning(std::string group_name)
        {
            ob::StateSpacePtr Joint_space(new ob::RealVectorStateSpace(motoman_arm_DOF));
            ob::RealVectorBounds Joint_bounds(motoman_arm_DOF);

            //Joint_bounds()
            setStateSpaceLimits(Joint_bounds);
            Joint_space->as<ob::RealVectorStateSpace>()->setBounds(Joint_bounds);

            // Setup Sample space
            ob::SpaceInformationPtr sample_si(new ob::SpaceInformation(Joint_space));

            //Setup Validity Checker
            ValidityChecker checker(sample_si) ;
            ompl::base::StateValidityCheckerPtr validity_checker(&checker, dealocate_StateValidityChecker_fn);
            sample_si->setStateValidityChecker(validity_checker);
            sample_si->setStateValidityCheckingResolution(0.01); // 1%
            sample_si->setup();

            //Set start and end state
            ob::ScopedState<> start_state = SetStateConfig(start_Config,sample_si);
            start_state.print(std::cout);
            ob::ScopedState<> goal_state= SetStateConfig(goal_Config,sample_si);
            goal_state.print(std::cout);

            // Creat Problem Definition
            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(sample_si));
            pdef->setStartAndGoalStates(start_state,goal_state);

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
            ob::PlannerStatus solved = planner->solve(planning_time);
            if(solved)
            {
                // Generate the Path
                path = pdef->getSolutionPath();
                std::cout<<"Path Found"<<std::endl;
                path->print(std::cout);
                std::cout<<"Total Cost: "<<path->cost(cost_fn).value()<<std::endl;
                std::cout<<"Total Length:"<<path->length();

                //std::cout<< "The path cost is:" <<path->cost()<<std::endl;
                //path->cost()
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
//         bool isStateValid(const ob::State* state)
//         {
//             // Moveit is needed here
//             const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
//             const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
//             const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

//             return (const void*)rot != (const void*)pos;
//         }

    public:
         float planning_time;
         double goal_tolerance;
         planner_type planner_choice;
         float cost_bias;

         std::vector<double> start_Config;
         std::vector<double> goal_Config;

         ob::PathPtr path;
         motoman_move_group m_robot_model;
    };

}


