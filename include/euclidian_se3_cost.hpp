
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "motoman_moveit.hpp"
using namespace ompl;

namespace optimize_planner
{
    class SE3dis_OptimizationObjective : public ompl::base::StateCostIntegralObjective
    {
    public:
        SE3dis_OptimizationObjective(const ompl::base::SpaceInformationPtr &si, motoman_move_group* plan_group,std::string& planning_group_name) : ompl::base::StateCostIntegralObjective(si,true)
        {
            planning_group = plan_group;
            group_name = planning_group_name;
        }
        virtual ~SE3dis_OptimizationObjective(){}

        virtual ompl::base::Cost stateCost(const ompl::base::State *s) const
        {
            // If in collision, then current cost should be NAN
            return ompl::base::Cost(si_->getStateValidityChecker()->clearance(s));
        }
        virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
        {
            // Calculate the euclidian distance is SE3 space between two state s1/s2

            double motion_cost = 0.0;
            std::vector<double> Jnt_config_1 = GetJointGroupValues(s1);
            std::vector<double> Jnt_config_2 = GetJointGroupValues(s2);

            double euclidean_dis = planning_group->get_euclidean_distance(group_name,Jnt_config_1,Jnt_config_2);

            //Eigen::Quaternion
            Eigen::Affine3d trans_1 = planning_group->get_ee_affine3dTransformation(group_name,Jnt_config_1);
            Eigen::Quaterniond rotation_1(trans_1.rotation());

            Eigen::Affine3d trans_2 = planning_group->get_ee_affine3dTransformation(group_name,Jnt_config_2);
            Eigen::Quaterniond rotation_2(trans_2.rotation());

            double angle_diff = get_quaternion_distance(rotation_1,rotation_2);

            motion_cost = euclidean_dis + 4*angle_diff;

            return ompl::base::Cost(motion_cost);
        }

        double get_quaternion_distance(Eigen::Quaterniond &q1, Eigen::Quaterniond &q2) const
        {
            double distance;

            double inner_product;

            inner_product = q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z()+q1.w()*q2.w();

            // When q1 == q2 the distance should be 0
            // When q1 is pointing the opposite direction to q2, the distance should be 1

            distance = acos(2*inner_product*inner_product - 1);
            //distance = 2*(1 - inner_product) ;
            //distance = 2*acos(inner_product);

            return distance;
        }

        std::vector<double> GetJointGroupValues(const ompl::base::State *state) const
        {
            std::vector<double> JntGrp_Val;
            for(int i = 0; i< motoman_arm_DOF; i++)
            {
                double joint_val;
                joint_val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
                JntGrp_Val.push_back(joint_val);
            }
            return JntGrp_Val;
        }
    private:
        motoman_move_group* planning_group;
        std::string group_name;
    };
}

