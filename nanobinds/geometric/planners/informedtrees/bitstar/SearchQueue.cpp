#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/bitstar/SearchQueue.h"
#include "../../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesBitstar_SearchQueue(nb::module_& m)
{
    // TODO [og::BITstar::SearchQueue][IMPLEMENT]
    nb::class_<og::BITstar::SearchQueue>(m, "SearchQueue")
        ;
}
