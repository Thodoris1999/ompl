#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_StateCostIntegralObjective(nb::module_ &m)
{
    struct PyStateCostIntegralObjective : ob::StateCostIntegralObjective
    {
        NB_TRAMPOLINE(ob::StateCostIntegralObjective, 5);

        // Optional override
        ob::Cost stateCost(const ob::State *s) const override
        {
            NB_OVERRIDE(stateCost, s);
        }

        // Optional override
        ob::Cost motionCost(const ob::State *s1, const ob::State* s2) const override
        {
            NB_OVERRIDE(motionCost, s1, s2);
        }

        // Optional override
        ob::Cost motionCostBestEstimate(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostBestEstimate, s1, s2);
        }
    };

    // TODO [ob::StateCostIntegralObjective][TEST]
    nb::class_<ob::StateCostIntegralObjective, PyStateCostIntegralObjective /* <-- trampoline */>(m, "StateCostIntegralObjective")
        // constructor
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("enableMotionCostInterpolation"))
        // getter
        .def("isMotionCostInterpolationEnabled", &ob::StateCostIntegralObjective::isMotionCostInterpolationEnabled);
}
