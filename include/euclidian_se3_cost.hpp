#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <moveit/robot_state/robot_state.h>

using namespace ompl;

class SE3dis_OptimizationObjective : public ompl::base::StateCostIntegralObjective
{
public:
    SE3dis_OptimizationObjective(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateCostIntegralObjective(si,true){}
    virtual ~SE3dis_OptimizationObjective(){}

    virtual ompl::base::Cost stateCost(const ompl::base::State *s) const
    {
        // If in collision, then current cost should be NAN
        return ompl::base::Cost(si_->getStateValidityChecker()->clearance(s));
    }
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
    {
        // Calculate the euclidian distance is SE3 space between two state s1/s2

    }

};
