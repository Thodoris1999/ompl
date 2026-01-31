#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/control/SimpleSetup.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/PathControl.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/OptimizationObjective.h"

#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

struct SimpleSetupPublicist : public oc::SimpleSetup {
    using oc::SimpleSetup::configured_;
};

void ompl::binding::control::init_SimpleSetup(nb::module_ &m)
{
    nb::class_<oc::SimpleSetup>(m, "SimpleSetup")
        // Constructors
        .def(nb::init<oc::SpaceInformationPtr>(), nb::arg("spaceInfo"))
        .def(nb::init<oc::ControlSpacePtr>(), nb::arg("controlSpace"))

        // getSpaceInformation, returns SpaceInformationPtr
        .def("getSpaceInformation", &oc::SimpleSetup::getSpaceInformation)

        // ProblemDefinition
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)()>(&oc::SimpleSetup::getProblemDefinition))
        .def("getProblemDefinitionConst",
             static_cast<const ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)() const>(
                 &oc::SimpleSetup::getProblemDefinition))

        // getStateSpace, getControlSpace
        .def("getStateSpace", &oc::SimpleSetup::getStateSpace)
        .def("getControlSpace", &oc::SimpleSetup::getControlSpace)

        // Common getters
        .def("getStateValidityChecker", &oc::SimpleSetup::getStateValidityChecker)
        .def("getStatePropagator", &oc::SimpleSetup::getStatePropagator)
        .def("getGoal", &oc::SimpleSetup::getGoal)
        .def("getPlanner", &oc::SimpleSetup::getPlanner)
        .def("getPlannerAllocator", &oc::SimpleSetup::getPlannerAllocator)

        // solution path checks
        .def("haveExactSolutionPath", &oc::SimpleSetup::haveExactSolutionPath)
        .def("haveSolutionPath", &oc::SimpleSetup::haveSolutionPath)

        // getSolutionPath: returns a PathControl& => use reference_internal
        .def(
            "getSolutionPath", [](oc::SimpleSetup &self) -> oc::PathControl & { return self.getSolutionPath(); })

        // getPlannerData
        .def(
            "getPlannerData", [](const oc::SimpleSetup &ss, ob::PlannerData &pd) { ss.getPlannerData(pd); },
            nb::arg("plannerData"))

        // setStateValidityChecker (two overloads)
        .def("setStateValidityChecker", nb::overload_cast<const ompl::base::StateValidityCheckerFn &>(
                                            &ompl::control::SimpleSetup::setStateValidityChecker))
        .def("clearStateValidityChecker", [](oc::SimpleSetup &ss) {
            ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(nullptr));
        })
        // setStatePropagator
        .def(
            "setStatePropagator",
            static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorFn &)>(&oc::SimpleSetup::setStatePropagator))
        .def("setStatePropagator",
             static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorPtr &)>(
                 &oc::SimpleSetup::setStatePropagator))

        // setOptimizationObjective
        .def("setOptimizationObjective", &oc::SimpleSetup::setOptimizationObjective, nb::arg("objective"))

        // start/goal states
        .def(
            "setStartAndGoalStates",
            [](oc::SimpleSetup &ss, const ob::State *start, const ob::State *goal, double threshold)
            {
                ss.getProblemDefinition()->setStartAndGoalStates(start, goal, threshold);
                ss.getProblemDefinition()->clearSolutionPaths();
                static_cast<SimpleSetupPublicist&>(ss).configured_ = false;
            },
            nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "setGoalState",
            [](oc::SimpleSetup &ss, const ob::State *goal, double threshold)
            {
                ss.getProblemDefinition()->setGoalState(goal, threshold);
                static_cast<SimpleSetupPublicist&>(ss).configured_ = false;
            },
            nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "addStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                ss.getProblemDefinition()->addStartState(state);
            },
            nb::arg("state"))
        .def("clearStartStates", &oc::SimpleSetup::clearStartStates)
        .def(
            "setStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                ss.clearStartStates();
                ss.getProblemDefinition()->addStartState(state);
            },
            nb::arg("state"))
        .def("setGoal", &oc::SimpleSetup::setGoal, nb::arg("goal"))

        // setPlanner, setPlannerAllocator
        .def("setPlanner", &oc::SimpleSetup::setPlanner, nb::arg("planner"))
        .def("setPlannerAllocator", &oc::SimpleSetup::setPlannerAllocator, nb::arg("allocator"))

        // solve() methods (two overloads)
        .def("solve", nb::overload_cast<double>(&oc::SimpleSetup::solve), nb::arg("time") = 1.0)
        .def("solve", nb::overload_cast<const ob::PlannerTerminationCondition &>(&oc::SimpleSetup::solve),
             nb::arg("terminationCondition"))

        // getLastPlannerStatus, getLastPlanComputationTime
        .def("getLastPlannerStatus", &oc::SimpleSetup::getLastPlannerStatus)
        .def("getLastPlanComputationTime", &oc::SimpleSetup::getLastPlanComputationTime)

        // clear
        .def("clear", &oc::SimpleSetup::clear)

        // print
        .def(
            "print",
            [](const oc::SimpleSetup &ss)
            {
                std::ostringstream oss;
                ss.print(oss);
                return oss.str();
            })

        // setup
        .def("setup", &oc::SimpleSetup::setup);
}
