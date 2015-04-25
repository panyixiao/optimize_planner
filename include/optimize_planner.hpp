// ROS
#include <ros/ros.h>
// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

//OMPL Planners
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//MOVEIT
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/world.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/allvalid/collision_world_allvalid.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

// robot model
//#include "optimize_planner/include/euclidian_se3_cost.hpp"

// Cost Function
#include "euclidian_se3_cost.hpp"
#include "cspace_cost.hpp"

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
boost::shared_ptr<std::string> plan_group ;

void dealocate_StateValidityChecker_fn(ompl::base::StateValidityChecker* p)
{
    //std::cout << ">>>>>>>>>>>>>>>Deallocate>>>>>>>>>>>>" << std::endl ;
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
    std::unique_ptr<planning_scene::PlanningScene> planning_scene_ptr_ ;
    boost::shared_ptr<collision_detection::CollisionRobot> crobot_;
    boost::shared_ptr<collision_detection::CollisionWorld> cworld_;
    std::unique_ptr<collision_detection::CollisionWorldAllValid> all_world_ptr_ ;
    collision_detection::AllowedCollisionMatrixPtr acm_ ;
    ros::NodeHandle nh;
    boost::shared_ptr<ros::Publisher> state_pub_ ;

public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si)
    {
        cworld_.reset() ;
        cworld_ = boost::shared_ptr<collision_detection::CollisionWorld>(new collision_detection::CollisionWorldFCL()) ;

        planning_scene_ptr_.reset() ;
        planning_scene_ptr_ = std::unique_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(rmodel_->getModel(),cworld_->getWorld())) ;

        crobot_.reset() ;
        crobot_ = boost::shared_ptr<collision_detection::CollisionRobotFCL>(new collision_detection::CollisionRobotFCL(rmodel_->getModel())) ;

        all_world_ptr_.reset() ;
        all_world_ptr_ = std::unique_ptr<collision_detection::CollisionWorldAllValid>(new collision_detection::CollisionWorldAllValid(cworld_->getWorld())) ;

        acm_.reset(new collision_detection::AllowedCollisionMatrix(rmodel_->getModel()->getLinkModelNames(),true)) ;
        //std::cout << acm_->hasEntry("hand_left_palm") << std::endl ;

        state_pub_.reset() ;
        state_pub_ = boost::shared_ptr<ros::Publisher>(new ros::Publisher) ;
        *state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1) ;

        std::vector<std::string> objs ;
        Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(1,0,0))) ;
        Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(0,1,0))) ;
        Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1))) ;
        Eigen::Affine3d r = rz * ry * rx ;
        Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1.28,0,0.885))) ;

        Eigen::Affine3d pose = r*t ;

        shapes::ShapePtr world_cube ;
        world_cube.reset(new shapes::Box(0.87, 0.87, 1.77));
        cworld_->getWorld()->addToObject("bin",world_cube,pose) ;

        specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::NONE;
        specs_.hasValidDirectionComputation = false;
    }

    //Function to check if state is valid
    virtual bool isValid(const ob::State *state) const
    {
        //std::cout << ">>>>>>>>>>>>>>>Validity Check>>>>>>>>>>>>" << std::endl ;

        std::vector<double> JointValues(motoman_arm_DOF) ;
        std::vector<std::string> variable_names(motoman_arm_DOF) ;

        const ob::RealVectorStateSpace::StateType *sample_state = state->as<ob::RealVectorStateSpace::StateType>() ;

        robot_state::RobotState& current_state = planning_scene_ptr_->getCurrentStateNonConst();
        const robot_model::JointModelGroup* model_group = current_state.getJointModelGroup(*plan_group);

        for( int i = 0 ; i < model_group->getJointModelNames().size() ; i++ )
        {
            variable_names[i] = model_group->getJointModelNames()[i] ;
            JointValues[i] = sample_state->values[i] ;
        }

        current_state.setVariablePositions(variable_names,JointValues) ;
        current_state.update() ;

        collision_detection::CollisionRequest req;
        req.group_name = *plan_group ;
        collision_detection::CollisionResult  res;
        all_world_ptr_->checkRobotCollision(req , res,*crobot_, current_state , *acm_ ) ;

        if(planning_scene_ptr_->isStateValid(current_state, *plan_group) == 1 &&
           current_state.satisfiesBounds(model_group) == 1 &&
           planning_scene_ptr_->isStateColliding(current_state, *plan_group) == 0)
        {
            return true  ;
        }
        else
        {
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
         }

        bool start_planning(std::string group_name)
        {
            std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

            plan_group.reset() ;
            plan_group = boost::shared_ptr<std::string>(new std::string) ;
            *plan_group = group_name ;

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
            //ob::OptimizationObjectivePtr path_length_cost_fn(new ob::PathLengthOptimizationObjective(sample_si));
            ob::OptimizationObjectivePtr path_length_cost_fn(new optimize_planner::Configuration_Space_cost(sample_si));
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
                std::cout<<"Path Found"<<std::endl;
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


