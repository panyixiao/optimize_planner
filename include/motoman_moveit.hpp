// ROS
#include <ros/ros.h>

// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Move Group
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

// Manipulator DOF
#define motoman_arm_DOF 7

// motoman_joint_limits
#define Joint_1_s_Limits 3.13
#define Joint_2_l_Limits 1.90
#define Joint_3_e_Limits 2.95
#define Joint_4_u_Limits 2.36
#define Joint_5_r_Limits 3.13
#define Joint_6_b_Limits 1.90
#define Joint_7_t_Limits 3.13

namespace optimize_planner
{
    class motoman_move_group
    {
    public:
        motoman_move_group()
        {
            robot_loader = new robot_model_loader::RobotModelLoader("robot_description");
            kinematic_model = robot_loader->getModel();
            kinematic_state = new robot_state::RobotState(kinematic_model);
            right_arm_joint_group = kinematic_model->getJointModelGroup("arm_right");
            left_arm_joint_group = kinematic_model->getJointModelGroup("arm_left");

            right_arm_group = new move_group_interface::MoveGroup("arm_right");
            left_arm_group = new move_group_interface::MoveGroup("arm_left");
        }

        std::vector<double> GetGroupConfig(std::string& groupName)
        {
            std::vector<double> JointValues;
            if (groupName == right_arm_group->getName())
            {
                JointValues = right_arm_group->getCurrentJointValues();
            }
            else if(groupName == left_arm_group->getName())
            {
                JointValues = left_arm_group->getCurrentJointValues();
            }
            else
            {
                ROS_INFO("Invalid GroupName");
            }

            return JointValues;
        }

        double get_ee_distance(std::string group_name,std::vector<double>& config_1, std::vector<double>& config_2)
        {
            geometry_msgs::PoseStamped position_1;
            geometry_msgs::PoseStamped position_2;

            if(group_name == "arm_right")
            {
                kinematic_state->setJointGroupPositions(right_arm_joint_group,config_1);
                kinematic_state->update();
                position_1 = right_arm_group->getCurrentPose(right_arm_group->getEndEffector());

                kinematic_state->setJointGroupPositions(right_arm_joint_group,config_2);
                kinematic_state->update();
                position_2 = right_arm_group->getCurrentPose(right_arm_group->getEndEffector());
            }
            else
            {
                kinematic_state->setJointGroupPositions(left_arm_joint_group,config_1);
                kinematic_state->update();
                position_1 = left_arm_group->getCurrentPose(left_arm_group->getEndEffector());

                kinematic_state->setJointGroupPositions(left_arm_joint_group,config_2);
                kinematic_state->update();
                position_2 = left_arm_group->getCurrentPose(left_arm_group->getEndEffector());
            }

            return pose_distance(position_1,position_2);
        }

    private:
        double pose_distance(geometry_msgs::PoseStamped& position_1,geometry_msgs::PoseStamped& position_2)
        {
            double x_diff = position_1.pose.position.x-position_2.pose.position.x;
            double y_diff = position_1.pose.position.y-position_2.pose.position.y;
            double z_diff = position_1.pose.position.z-position_2.pose.position.z;
            return sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
        }

    public:
        move_group_interface::MoveGroup* right_arm_group;
        move_group_interface::MoveGroup* left_arm_group;

    private:
        robot_model_loader::RobotModelLoader* robot_loader;
        robot_model::RobotModelConstPtr kinematic_model;
        robot_state::RobotState* kinematic_state;

        const robot_model::JointModelGroup* right_arm_joint_group;
        const robot_model::JointModelGroup* left_arm_joint_group;

    };
}
