#ifndef MOTOMAN_MOVEIT_HPP
#define MOTOMAN_MOVEIT_HPP

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
#define Joint_torso_Limits 2.95
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

boost::shared_ptr<robot_model_loader::RobotModelLoader> rmodel_ ;

namespace optimize_planner
{
    class motoman_move_group
    {
    public:
        motoman_move_group()
        {
            marker_pub_.reset() ;
            marker_pub_ = boost::shared_ptr<ros::Publisher>(new ros::Publisher) ;
            *marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

            rmodel_.reset() ;
            rmodel_ = boost::shared_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description")) ;
            kinematic_model = rmodel_->getModel() ;
            kinematic_state = new robot_state::RobotState(kinematic_model);

            right_arm_joint_group = kinematic_model->getJointModelGroup("arm_right");
            left_arm_joint_group = kinematic_model->getJointModelGroup("arm_left");

            right_arm_joint_torso_group = kinematic_model->getJointModelGroup("arm_right_torso");
            left_arm_joint_torso_group = kinematic_model->getJointModelGroup("arm_left_torso");

            right_arm_group = new move_group_interface::MoveGroup("arm_right");
            left_arm_group = new move_group_interface::MoveGroup("arm_left");

            right_arm_torso_group = new move_group_interface::MoveGroup("arm_right_torso");
            left_arm_torso_group = new move_group_interface::MoveGroup("arm_left_torso");

            display_color_scale = 0.0;
            //traj_manager = new traj_man::TrajectoryExecutionManager(kinematic_model);
        }

        std::vector<double> GetGroupCurrentConfig(std::string& groupName)
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
            if (groupName == right_arm_torso_group->getName())
            {
                JointValues = right_arm_torso_group->getCurrentJointValues();
            }
            else if(groupName == left_arm_torso_group->getName())
            {
                JointValues = left_arm_torso_group->getCurrentJointValues();
            }
            else
            {
                ROS_INFO("Invalid GroupName");
            }
            return JointValues;
        }

        double get_euclidean_distance(std::string group_name,std::vector<double>& config_1, std::vector<double>& config_2)
        {
            geometry_msgs::PoseStamped position_1;
            geometry_msgs::PoseStamped position_2;
            Eigen::Affine3d ee_transformation;

            ee_transformation = get_ee_affine3dTransformation(group_name,config_1);
            position_1 = getTranslationPoseCoord(ee_transformation);

            ee_transformation = get_ee_affine3dTransformation(group_name,config_2);
            position_1 = getTranslationPoseCoord(ee_transformation);

            return pose_distance(position_1,position_2);
        }

        Eigen::Affine3d get_ee_affine3dTransformation(std::string group_name,std::vector<double>& config)
        {
            Eigen::Affine3d transform_matrix;
            move_group_interface::MoveGroup* group_pointer;
            const robot_model::JointModelGroup* joint_group_pointer ;

            if(group_name == "arm_right")
            {
                group_pointer = right_arm_group;
                joint_group_pointer = &(*right_arm_joint_group);
            }
            else if(group_name == "arm_left")
            {
                group_pointer = left_arm_group;
                joint_group_pointer = left_arm_joint_group;
            }
            else if(group_name == "arm_right_torso")
            {
                group_pointer = right_arm_torso_group;
                joint_group_pointer = right_arm_joint_torso_group;
            }
            else
            {
                group_pointer = left_arm_torso_group;
                joint_group_pointer = left_arm_joint_torso_group;
            }

            kinematic_state->setJointGroupPositions(joint_group_pointer,config);
            kinematic_state->update();
            transform_matrix = kinematic_state->getGlobalLinkTransform(group_pointer->getEndEffectorLink());
            return transform_matrix;
        }

        geometry_msgs::PoseStamped getTranslationPoseCoord(Eigen::Affine3d& transform)
        {
            geometry_msgs::PoseStamped translation;
            translation.pose.position.x = transform.matrix().data()[12];
            translation.pose.position.y = transform.matrix().data()[13];
            translation.pose.position.z = transform.matrix().data()[14];

            return translation;
        }

        bool generate_trajectory(std::vector< std::vector<double> > &joint_trajectory,std::string group_name)
        {
            double start_time = 0.5;
            int arm_dof = motoman_arm_DOF;
            std::vector<std::string> JointNames;

            if(group_name == "arm_right")
            {
                JointNames = right_arm_group->getJoints();
            }
            else if(group_name == "arm_left")
            {
                JointNames = left_arm_group->getJoints();
            }
            else if(group_name == "arm_right_torso")
            {
                JointNames = right_arm_torso_group->getJoints();
                arm_dof = 8;
            }
            else
            {
                JointNames = left_arm_torso_group->getJoints();
                arm_dof = 8;
            }

            m_trajectory.joint_trajectory.joint_names.resize(arm_dof);
            for(int k = 0; k< arm_dof;k++)
            {
                //std::cout<<"Joint: "<<JointNames[k]<<std::endl;
                m_trajectory.joint_trajectory.joint_names[k] = JointNames[k];
            }

            int trajectory_length = joint_trajectory.size();
            m_trajectory.joint_trajectory.points.resize(trajectory_length+1);
            std::vector<double> current_joint_value = GetGroupCurrentConfig(group_name);
            // Init first point
            m_trajectory.joint_trajectory.points[0].positions.resize(arm_dof);
            m_trajectory.joint_trajectory.points[0].velocities.resize(arm_dof);
            for(int i = 0; i<arm_dof;i++)
            {
                m_trajectory.joint_trajectory.points[0].positions[i] = current_joint_value[i];
                m_trajectory.joint_trajectory.points[0].velocities[i] = 0.0;
            }

            m_trajectory.joint_trajectory.points[0].time_from_start = ros::Duration(start_time);
            // Fill the rest of the trajectory
            double time_from_start  = start_time;

            for(int i = 0; i<trajectory_length;i++)
            {
                m_trajectory.joint_trajectory.points[i+1].positions.resize(arm_dof);
                m_trajectory.joint_trajectory.points[i+1].velocities.resize(arm_dof);
                for(int j = 0; j<arm_dof;j++)
                {
                    m_trajectory.joint_trajectory.points[i+1].positions[j] = joint_trajectory[i][j];
                    //std::cout<<"Point "<<i+1<<", Joint "<<j<<" config: "<<joint_trajectory[i][j]<<std::endl;
                    m_trajectory.joint_trajectory.points[i+1].velocities[j] = 0.0;
                }


                double max_joint_move = 0;
                for(int j = 0; j< arm_dof; j++)
                {
                    double joint_move = fabs(m_trajectory.joint_trajectory.points[i+1].positions[j] - m_trajectory.joint_trajectory.points[i].positions[j]);
                    if(joint_move > max_joint_move)
                    {
                        max_joint_move = joint_move;
                    }
                }

                double seconds = max_joint_move/MAX_JOINT_VEL;

                time_from_start += seconds;

                m_trajectory.joint_trajectory.points[i+1].time_from_start = ros::Duration(time_from_start);
            }

            m_trajectory.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(start_time);

            std::cout<<"Totally "<<m_trajectory.joint_trajectory.points.size()<<" points will be executed"<<std::endl;

            return true;
        }

        void display_traj(trajectory_msgs::JointTrajectory traj , std::string group_name)
        {
            ROS_INFO("Displaying Trajectory...") ;
            geometry_msgs::PoseStamped ee_pos ;
            std::vector<std::string> JointNames;

            std_msgs::ColorRGBA point_color, line_color;
            point_color.r = 1.0f;
            point_color.g = 1.0f;
            point_color.b = 0.0f;
            point_color.a = 1.0;

            line_color.r = float(1 - display_color_scale);
            line_color.g = float(display_color_scale);
            line_color.b = 0.0f;
            line_color.a = 1.0;

            int arm_dof = motoman_arm_DOF;

            move_group_interface::MoveGroup* group_pointer;

            if(group_name == "arm_right")
            {
                group_pointer = right_arm_group;
            }
            else if(group_name == "arm_left")
            {
                group_pointer = left_arm_group;
            }
            else if(group_name == "arm_right_torso")
            {
                group_pointer = right_arm_torso_group;
                arm_dof = 8;
            }
            else
            {
                group_pointer = left_arm_torso_group;
                arm_dof = 8;
            }

            JointNames = group_pointer->getJoints();
            // There three customized joint added which is not bolongs to original model

            std::vector<double> joint_pos = GetGroupCurrentConfig(group_name);
            Eigen::Affine3d ee_transformation;
            visualization_msgs::Marker point ;
            visualization_msgs::Marker line ;

            for( int i=0 ; i< traj.points.size() ; i++)
            {
                for(int j = 0; j<arm_dof;j++)
                {
                    joint_pos[j] = traj.points[i].positions[j];
                }

                kinematic_state->setVariablePositions(JointNames,joint_pos);
                kinematic_state->update();

                ee_transformation = kinematic_state->getGlobalLinkTransform(group_pointer->getEndEffectorLink());
                // Get the translate column
                geometry_msgs::Point p ;
                p.x = ee_pos.pose.position.x = ee_transformation.matrix().data()[12];
                p.y = ee_pos.pose.position.y = ee_transformation.matrix().data()[13];
                p.z = ee_pos.pose.position.z = ee_transformation.matrix().data()[14];


                //point.header.frame_id = "optimize_planner" ;
                point.header.frame_id = "base_link";
                line.header.frame_id = "base_link" ;

                point.header.stamp = ros::Time::now();
                line.header.stamp = ros::Time::now() ;

                point.ns = "point" ;
                point.id = i+1 ;
                line.ns = "path" ;
                line.id = i+51 ;

                point.type = visualization_msgs::Marker::SPHERE_LIST ;
                point.action = visualization_msgs::Marker::ADD ;
                line.type = visualization_msgs::Marker::LINE_STRIP ;
                line.action = visualization_msgs::Marker::ADD ;

                point.lifetime = ros::Duration(60);
                line.lifetime = ros::Duration(60);

                point.scale.x = 0.02 ;
                point.scale.y = 0.02 ;
                point.scale.z = 0.02 ;

                line.scale.x = 0.02 ;
                line.scale.y = 0.02 ;
                line.scale.z = 0.02 ;

                point.color = point_color;
                line.color = line_color ;

                //std::cout<<"Marker Position: "<<"x: "<<p.x<<"y: "<<p.y<<"z: "<<p.z<<std::endl;
                point.points.push_back(p);
                line.points.push_back(p);
            }
            marker_pub_->publish(point);
            marker_pub_->publish(line);

            ROS_INFO("Path Published !! ") ;
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

        move_group_interface::MoveGroup* right_arm_torso_group;
        move_group_interface::MoveGroup* left_arm_torso_group;

        moveit_msgs::RobotTrajectory m_trajectory;
        //trajectory_msgs::JointTrajectory m_trajectory;
        double display_color_scale;

    private:
        robot_model_loader::RobotModelLoader* robot_loader;
        robot_model::RobotModelConstPtr kinematic_model;
        robot_state::RobotState* kinematic_state;

        const robot_model::JointModelGroup* right_arm_joint_group;
        const robot_model::JointModelGroup* left_arm_joint_group;

        const robot_model::JointModelGroup* right_arm_joint_torso_group;
        const robot_model::JointModelGroup* left_arm_joint_torso_group;

        traj_man::TrajectoryExecutionManager* traj_manager;
        ros::NodeHandle nh;
        boost::shared_ptr<ros::Publisher> marker_pub_ ;
    };
}

#endif
