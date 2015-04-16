// ROS
#include <ros/ros.h>

// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Move Group
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

// Trajetory
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <actionlib/client/action_client.h>

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

// motoman_joint_velocity
#define MAX_JOINT_VEL 2.8   // Unit Rad/s


namespace traj_man = trajectory_execution_manager;

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

            traj_manager = new traj_man::TrajectoryExecutionManager(kinematic_model);
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

        bool execute_joint_trajectory(std::vector<double*> joint_trajectory,std::string group_name)
        {
            double start_time = 0.5;

            std::vector<std::string> JointNames;
            if(group_name == "arm_right")
            {
                JointNames = right_arm_group->getJoints();
            }
            else
            {
                JointNames = left_arm_group->getJoints();
            }
            m_trajectory.joint_names.resize(motoman_arm_DOF);
            for(int k = 0; k< motoman_arm_DOF;k++)
            {
                std::cout<<"Joint: "<<JointNames[k]<<std::endl;
                m_trajectory.joint_names[k] = JointNames[k];
            }

            int trajectory_length = joint_trajectory.size();
            m_trajectory.points.resize(trajectory_length+1);
            std::vector<double> current_joint_value = GetGroupConfig(group_name);
            // Init first point
            m_trajectory.points[0].positions.resize(motoman_arm_DOF);
            m_trajectory.points[0].velocities.resize(motoman_arm_DOF);
            for(int i = 0; i<motoman_arm_DOF;i++)
            {
                m_trajectory.points[0].positions[i] = current_joint_value[i];
                m_trajectory.points[0].velocities[i] = 0.0;
            }
            m_trajectory.points[0].time_from_start = ros::Duration(start_time);
            // Fill the rest of the trajectory
            double time_from_start  = start_time;

            for(int i = 0; i<trajectory_length;i++)
            {
                m_trajectory.points[i+1].positions.resize(motoman_arm_DOF);
                m_trajectory.points[i+1].velocities.resize(motoman_arm_DOF);
                for(int j = 0; j<motoman_arm_DOF;j++)
                {
                    m_trajectory.points[i+1].positions[j] = joint_trajectory[i][j];
                    m_trajectory.points[i+1].velocities[j] = 0.0;
                }

                double max_joint_move = 0;
                for(int j = 0; j< motoman_arm_DOF; j++)
                {
                    double joint_move = fabs(m_trajectory.points[i+1].positions[j] - m_trajectory.points[i].positions[j]);
                    if(joint_move > max_joint_move)
                    {
                        max_joint_move = joint_move;
                    }
                }
                double seconds = max_joint_move/MAX_JOINT_VEL;

                time_from_start+=seconds;

                m_trajectory.points[i+1].time_from_start = ros::Duration(time_from_start);
            }

            m_trajectory.header.stamp = ros::Time::now() + ros::Duration(start_time);

            ROS_INFO("Sending Trajectory to joint_trajectory_action");

            traj_manager->pushAndExecute(m_trajectory);
            traj_manager->waitForExecution();


            return true;
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

        traj_man::TrajectoryExecutionManager* traj_manager;
        trajectory_msgs::JointTrajectory m_trajectory;

    };
}
