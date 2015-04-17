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

        //traj_manager = new traj_man::TrajectoryExecutionManager(kinematic_model);
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
        Eigen::Affine3d ee_transformation;

        if(group_name == "arm_right")
        {
            kinematic_state->setJointGroupPositions(right_arm_joint_group,config_1);
            kinematic_state->update();
            ee_transformation = kinematic_state->getGlobalLinkTransform(right_arm_group->getEndEffectorLink());
            position_1 = getTranslationPoseCoord(ee_transformation);

            kinematic_state->setJointGroupPositions(right_arm_joint_group,config_2);
            kinematic_state->update();
            ee_transformation = kinematic_state->getGlobalLinkTransform(right_arm_group->getEndEffectorLink());
            position_2 = getTranslationPoseCoord(ee_transformation);
        }
        else
        {
            kinematic_state->setJointGroupPositions(left_arm_joint_group,config_1);
            kinematic_state->update();
            ee_transformation = kinematic_state->getGlobalLinkTransform(left_arm_group->getEndEffectorLink());
            position_1 = getTranslationPoseCoord(ee_transformation);

            kinematic_state->setJointGroupPositions(left_arm_joint_group,config_2);
            kinematic_state->update();
            ee_transformation = kinematic_state->getGlobalLinkTransform(left_arm_group->getEndEffectorLink());
            position_2 = getTranslationPoseCoord(ee_transformation);
        }
        return pose_distance(position_1,position_2);
    }

    geometry_msgs::PoseStamped getTranslationPoseCoord(Eigen::Affine3d& transform)
    {
        geometry_msgs::PoseStamped translation;
        translation.pose.position.x = transform.matrix().data()[12];
        translation.pose.position.y = transform.matrix().data()[13];
        translation.pose.position.z = transform.matrix().data()[14];

        return translation;
    }

    bool execute_joint_trajectory(std::vector< std::vector<double> > &joint_trajectory,std::string group_name)
    {
        double start_time = 0.5;

        std::vector<std::string> JointNames;
        std::vector<std::string> arm_controller;
        if(group_name == "arm_right")
        {
            JointNames = right_arm_group->getJoints();
            arm_controller.push_back("fake_arm_right_controller");
        }
        else
        {
            JointNames = left_arm_group->getJoints();
            arm_controller.push_back("fake_arm_left_controller");
        }
        m_trajectory.joint_names.resize(motoman_arm_DOF);
        for(int k = 0; k< motoman_arm_DOF;k++)
        {
            //std::cout<<"Joint: "<<JointNames[k]<<std::endl;
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
                //std::cout<<"Point "<<i+1<<", Joint "<<j<<" config: "<<joint_trajectory[i][j]<<std::endl;
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

        std::cout<<"Totally "<<m_trajectory.points.size()<<" points will be executed"<<std::endl;
        ROS_INFO("Sending Trajectory to joint_trajectory_action");
        //traj_manager->pushAndExecute(m_trajectory);

        ROS_INFO("Trajectory Sended....");
        display_traj(m_trajectory,group_name);
        return true;
    }

    void display_traj(trajectory_msgs::JointTrajectory traj , std::string group_name )
    {
        ROS_INFO("Displaying Trajectory") ;
        geometry_msgs::PoseStamped ee_pos ;
        std::vector<std::string> JointNames;
        ros::NodeHandle node_handle ;
        //ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, true);
        ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker",1);

        if( !marker_pub ) {
            ROS_INFO("Invalid Publisher !! ") ;
        }
        else
            ROS_INFO("Valid Publisher !! ") ;

        visualization_msgs::MarkerArray path ;
        std_msgs::ColorRGBA point_color;
        point_color.r = 1.0;
        point_color.g = 1.0;
        point_color.b = 0.0;
        point_color.a = 1.0;

        if(group_name == "arm_right")
        {
            JointNames = right_arm_group->getJoints();
        }
        else
        {
            JointNames = left_arm_group->getJoints();
        }
        // There three customized joint added which is not bolongs to original model
        JointNames.pop_back();
        JointNames.pop_back();
        JointNames.pop_back();

        std::vector<double> joint_pos = GetGroupConfig(group_name);
        Eigen::Affine3d ee_transformation;

        for( int i=0 ; i< traj.points.size() ; i++)
        {
            visualization_msgs::Marker point ;
            std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
            for(int j = 0; j<motoman_arm_DOF;j++)
            {
                joint_pos[j] = traj.points[i].positions[j];
                //std::cout<<"Joint "<<j<<" Position"<<traj.points[i].positions[j]<<std::endl;
            }

            kinematic_state->setVariablePositions(JointNames,joint_pos);
            kinematic_state->update();
            if(group_name == "arm_right")
            {
                ee_transformation = kinematic_state->getGlobalLinkTransform(right_arm_group->getEndEffectorLink());
            }
            else
            {
                ee_transformation = kinematic_state->getGlobalLinkTransform(left_arm_group->getEndEffectorLink());
            }

            // Get the translate column
            ee_pos.pose.position.x = ee_transformation.matrix().data()[12];
            ee_pos.pose.position.y = ee_transformation.matrix().data()[13];
            ee_pos.pose.position.z = ee_transformation.matrix().data()[14];


            //point.header.frame_id = "optimize_planner" ;
            point.header.frame_id = "base_link";
            point.header.stamp = ros::Time::now();
            point.ns = "path" ;
            point.id = i+1 ;
            point.type = visualization_msgs::Marker::CUBE_LIST ;
            point.action = visualization_msgs::Marker::ADD ;
            //point.lifetime = ros::Duration() ;
            //point.frame_locked = false ;
            point.scale.x = 1 ;
            point.scale.y = 1 ;
            point.scale.z = 1 ;
            point.pose.position.x = ee_pos.pose.position.x ;
            point.pose.position.y = ee_pos.pose.position.y ;
            point.pose.position.z = ee_pos.pose.position.z ;
            point.pose.orientation.x =ee_pos.pose.orientation.x ;
            point.pose.orientation.y = ee_pos.pose.orientation.y ;
            point.pose.orientation.z = ee_pos.pose.orientation.z ;
            point.pose.orientation.w = ee_pos.pose.orientation.w ;
            point.color = point_color;

            std::cout<<"Marker Position: "<<"x: "<<point.pose.position.x<<"y: "<<point.pose.position.y<<"z: "<<point.pose.position.z<<std::endl;
            marker_pub.publish(point);
            path.markers.push_back(point) ;
        }

        //marker_pub.publish(path) ;
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
