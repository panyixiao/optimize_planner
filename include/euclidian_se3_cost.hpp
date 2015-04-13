
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include"motoman_moveit.hpp"
using namespace ompl;

namespace optimize_planner
{
    class SE3dis_OptimizationObjective : public ompl::base::StateCostIntegralObjective
    {
    public:
        SE3dis_OptimizationObjective(const ompl::base::SpaceInformationPtr &si, motoman_move_group* robot_model,std::string& planning_group_name) : ompl::base::StateCostIntegralObjective(si,true)
        {
            planning_group = robot_model;
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
            //const ompl::base::State *state_1 = s1;
            std::vector<double> Jnt_config_1 = GetJointGroupValues(s1);
            std::vector<double> Jnt_config_2 = GetJointGroupValues(s2);

            double distance = planning_group->get_ee_distance(group_name,Jnt_config_1,Jnt_config_2);
            return ompl::base::Cost(distance);
        }

        std::vector<double> GetJointGroupValues(const ompl::base::State *state) const
        {
            std::vector<double> JntGrp_Val;
            for(int i = 0; i<motoman_arm_DOF;i++)
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

