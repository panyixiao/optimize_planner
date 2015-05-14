
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "motoman_moveit.hpp"

namespace ob = ompl::base;

namespace optimize_planner
{
    class Configuration_Space_cost :public ob::StateCostIntegralObjective
    {
    public:
        Configuration_Space_cost(const ob::SpaceInformationPtr &si, int c_space_size): ob::StateCostIntegralObjective(si)
        {
            space_dim = c_space_size;
        }

        virtual ob::Cost stateCost(const ob::State *s) const
        {
            // If in collision, then current cost should be NAN
            return ompl::base::Cost(si_->getStateValidityChecker()->clearance(s));
        }

        virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
        {
            double motion_cost = 0.0;
            std::vector<double> config_1 = GetJointGroupValues(s1);
            std::vector<double> config_2 = GetJointGroupValues(s2);

            std::vector<double> joint_weight = GenerateJointWeight(space_dim);

            motion_cost = get_motion_cost(config_1,config_2,joint_weight);

            return ob::Cost(motion_cost);
        }

        std::vector<double> GenerateJointWeight(int space_dim) const
        {
            std::vector<double> Joint_Weight;
            double MAX_VALUE = 10;
            if(space_dim == 8)
            {
                Joint_Weight.push_back(MAX_VALUE);
                Joint_Weight.push_back(MAX_VALUE);
                Joint_Weight.push_back(MAX_VALUE);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
            }
            else
            {
                Joint_Weight.push_back(MAX_VALUE);
                Joint_Weight.push_back(MAX_VALUE);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
                Joint_Weight.push_back(MAX_VALUE/100);
            }

            return Joint_Weight;
        }

    private:
        std::vector<double> GetJointGroupValues(const ob::State *state) const
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

        double get_motion_cost(std::vector<double> &c_1, std::vector<double> &c_2, std::vector<double> &weight) const
        {
            std::vector<double>::iterator iter_1 = c_1.begin();
            std::vector<double>::iterator iter_2 = c_2.begin();
            std::vector<double>::iterator iter_weight = weight.begin();

            double distance = 0.0;

            for(; iter_1 != c_1.end() &&
                  iter_2 != c_2.end() &&
                  iter_weight!=weight.end();
                  iter_1++, iter_2++,iter_weight++)
            {
                double JntVal_1 = *iter_1;
                double JntVal_2 = *iter_2;
                double JntWeight = *iter_weight;

                double diff = JntVal_2 - JntVal_1;
                distance += diff*diff*JntWeight;
            }

            distance = sqrt(distance);
            return distance;
        }

        int space_dim;
    };

}
