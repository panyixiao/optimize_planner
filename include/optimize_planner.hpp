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
    //    std::unique_ptr<planning_scene::PlanningScene> planning_scene_ptr_;
    //    std::unique_ptr<collision_detection::CollisionWorldAllValid> all_world_ptr_ ;
    //    //std::unique_ptr<collision_detection::CollisionWorld> world_ ;
    //    collision_detection::WorldPtr world_ptr = collision_detection::WorldPtr(new collision_detection::World()) ;
    //    std::unique_ptr<robot_model_loader::RobotModelLoader> rmodel;
    //    boost::shared_ptr<collision_detection::CollisionRobot> crobot_;
    std::unique_ptr<planning_scene::PlanningScene> planning_scene_ptr_ ;
    std::unique_ptr<robot_model_loader::RobotModelLoader> rmodel_ ;
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
        rmodel_.reset() ;
        rmodel_ = std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description")) ;

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

        //        //world_.reset(new collision_detection::CollisionWorldFCL() ) ;
        //        rmodel.reset(new robot_model_loader::RobotModelLoader("robot_description")) ;
        //        planning_scene_ptr_.reset();
        //        planning_scene_ptr_ = std::unique_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(rmodel->getModel(),world_ptr));
        //        all_world_ptr_.reset() ;
        //        all_world_ptr_ = std::unique_ptr<collision_detection::CollisionWorldAllValid>(new collision_detection::CollisionWorldAllValid(world_ptr)) ;
        //        crobot_.reset(new collision_detection::CollisionRobotFCL(rmodel)) ;

        //        moveit::planning_interface::MoveGroup group("arm_left") ;
        //        moveit_msgs::CollisionObject c_object ;
        //        c_object.header.frame_id = group.getPlanningFrame() ;

        //        c_object.id = "shelf" ;
        //        shape_msgs::SolidPrimitive bin ;
        //        bin.type = bin.BOX ;
        //        bin.dimensions.resize(3) ;
        //        bin.dimensions[0] = 0.87 ;
        //        bin.dimensions[1] = 0.87 ;
        //        bin.dimensions[2] = 1.77 ;

        //        geometry_msgs::Pose bin_pose ;
        //        bin_pose.orientation.x = 0 ;
        //        bin_pose.orientation.y = 0 ;
        //        bin_pose.orientation.z = 0.7071 ;
        //        bin_pose.orientation.w = 0.7071 ;
        //        bin_pose.position.x = 1.32 ;
        //        bin_pose.position.y = 0 ;
        //        bin_pose.position.z = 0 ;

        //        c_object.primitives.push_back(bin) ;
        //        c_object.primitive_poses.push_back(bin_pose) ;
        //        c_object.operation = c_object.ADD ;

        //        std::vector<moveit_msgs::CollisionObject> c_objs ;
        //        c_objs.push_back(c_object) ;
        //        moveit::planning_interface::PlanningSceneInterface p_s_interface ;
        //        p_s_interface.addCollisionObjects(c_objs) ;

        std::vector<std::string> objs ;
        Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(1,0,0))) ;
        Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(0,1,0))) ;
        Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1))) ;
        Eigen::Affine3d r = rz * ry * rx ;
        Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1.28,0,0.885))) ;

        Eigen::Affine3d pose = r*t ;
        std::cout << pose.matrix() << std::endl ;

        shapes::ShapePtr world_cube ;
        world_cube.reset(new shapes::Box(0.87, 0.87, 1.77));
        cworld_->getWorld()->addToObject("bin",world_cube,pose) ;

        objs = cworld_->getWorld()->getObjectIds() ;
        std::cout << "objects present " << objs.size() << std::endl ;
        if(objs.size())
        {
            for( int i=0 ; i<cworld_->getWorld()->size() ; i++)
                std::cout << objs[i] << std::endl ;
        }
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
        current_state.update() ;

        /* loop at 1 Hz */
        ros::Rate loop_rate(1);

        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(current_state, msg.state);

        /* send the message to the RobotState display */
        state_pub_->publish( msg );

        ros::spinOnce();
        loop_rate.sleep();

        Eigen::Affine3d endef = current_state.getGlobalLinkTransform("arm_left_link_7_t") ;
        std::cout << endef.matrix() << std::endl ;

        collision_detection::CollisionRequest req;
        req.group_name = "left_arm" ;
        collision_detection::CollisionResult  res;
        all_world_ptr_->checkRobotCollision(req , res,*crobot_, current_state , *acm_ ) ;

        std::cout << "state valid  ? " << planning_scene_ptr_->isStateValid(current_state, "left_arm") << std::endl ;
        //        std::cout << "state collid ? " << planning_scene_ptr_->isStateColliding(current_state,"left_arm") << std::endl ;
        std::cout << "state collid ? " << res.collision << std::endl ;
        std::cout << "state bound  ? " << current_state.satisfiesBounds(model_group) << std::endl ;
        std::cout << "******************************" << std::endl ;
        //        planning_scene_ptr_->printKnownObjects(std::cout) ;

        if(planning_scene_ptr_->isStateValid(current_state, "left_arm") == 1
                && current_state.satisfiesBounds(model_group) == 1
                && planning_scene_ptr_->isStateColliding(current_state,"left_arm") == 0)
        {
            //            std::cout << ">>>>>>>>>>>>>>>Valid State>>>>>>>>>>>>" << std::endl ;
            return true  ;
        }
        else
        {
            //            std::cout << ">>>>>>>>>>>>>>>Invalid State>>>>>>>>>>>>" << std::endl ;
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


